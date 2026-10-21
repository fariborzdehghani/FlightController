#include "srf05.h"
#include <math.h>

#define SRF05_TRIGGER_PULSE_US       10u
#define SRF05_MIN_TRIGGER_PERIOD_MS  50u
#define SRF05_DEFAULT_TIMEOUT_MS     60u
#define SRF05_US_PER_CM              58.0f

typedef enum {
  SRF05_CAPTURE_IDLE = 0,
  SRF05_CAPTURE_RISING,
  SRF05_CAPTURE_FALLING,
  SRF05_CAPTURE_COMPLETE
} SRF05_CaptureState_t;

static SRF05_Config_t config;
static uint8_t initialized;

static volatile SRF05_CaptureState_t capture_state = SRF05_CAPTURE_IDLE;
static volatile uint64_t rising_capture;
static volatile uint32_t timer_overflows;
static volatile uint64_t pulse_ticks;
static volatile uint32_t capture_complete_ms;

static uint8_t has_triggered;
static uint32_t last_trigger_ms;
static uint32_t measurement_start_ms;

static uint8_t srf05_channel_is_valid(uint32_t channel) {
  return (channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 ||
          channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4);
}

static uint8_t srf05_is_active_channel(uint32_t active_channel,
                                       uint32_t channel) {
  return ((active_channel == HAL_TIM_ACTIVE_CHANNEL_1 &&
           channel == TIM_CHANNEL_1) ||
          (active_channel == HAL_TIM_ACTIVE_CHANNEL_2 &&
           channel == TIM_CHANNEL_2) ||
          (active_channel == HAL_TIM_ACTIVE_CHANNEL_3 &&
           channel == TIM_CHANNEL_3) ||
          (active_channel == HAL_TIM_ACTIVE_CHANNEL_4 &&
           channel == TIM_CHANNEL_4));
}

static uint32_t srf05_capture_flag(uint32_t channel) {
  switch (channel) {
  case TIM_CHANNEL_1:
    return TIM_FLAG_CC1;
  case TIM_CHANNEL_2:
    return TIM_FLAG_CC2;
  case TIM_CHANNEL_3:
    return TIM_FLAG_CC3;
  case TIM_CHANNEL_4:
    return TIM_FLAG_CC4;
  default:
    return 0u;
  }
}

static uint32_t srf05_overcapture_flag(uint32_t channel) {
  switch (channel) {
  case TIM_CHANNEL_1:
    return TIM_FLAG_CC1OF;
  case TIM_CHANNEL_2:
    return TIM_FLAG_CC2OF;
  case TIM_CHANNEL_3:
    return TIM_FLAG_CC3OF;
  case TIM_CHANNEL_4:
    return TIM_FLAG_CC4OF;
  default:
    return 0u;
  }
}

static void srf05_delay_us(uint32_t delay_us) {
#if defined(DWT) && defined(CoreDebug) && \
    defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  const uint32_t cycles_per_us = SystemCoreClock / 1000000u;
  const uint32_t start = DWT->CYCCNT;
  const uint32_t delay_cycles = delay_us * cycles_per_us;

  while ((DWT->CYCCNT - start) < delay_cycles) {
    __NOP();
  }
#else
  volatile uint32_t cycles =
      delay_us * (SystemCoreClock / 1000000u) / 5u;
  while (cycles-- > 0u) {
    __NOP();
  }
#endif
}

static void srf05_stop_capture(void) {
  __HAL_TIM_DISABLE_IT(config.timer, TIM_IT_UPDATE);
  (void)HAL_TIM_IC_Stop_IT(config.timer, config.captureChannel);
  __HAL_TIM_SET_CAPTUREPOLARITY(config.timer, config.captureChannel,
                                TIM_INPUTCHANNELPOLARITY_RISING);
}

static uint64_t srf05_capture_ticks(uint32_t captured) {
  const uint64_t period =
      (uint64_t)__HAL_TIM_GET_AUTORELOAD(config.timer) + 1u;
  const uint32_t update_before =
      __HAL_TIM_GET_FLAG(config.timer, TIM_FLAG_UPDATE);
  uint32_t counter = __HAL_TIM_GET_COUNTER(config.timer);
  const uint32_t update_after =
      __HAL_TIM_GET_FLAG(config.timer, TIM_FLAG_UPDATE);

  if (update_before != update_after) {
    /* The counter wrapped while taking the snapshot. */
    counter = __HAL_TIM_GET_COUNTER(config.timer);
  }

  uint64_t ticks =
      ((uint64_t)timer_overflows + (update_after != RESET)) * period + captured;
  if (captured > counter) {
    /* The edge preceded the most recent wrap. That update may already have
     * been serviced: HAL snapshots SR before dispatching its callbacks. */
    ticks -= period;
  }
  return ticks;
}

static HAL_StatusTypeDef srf05_start_capture(void) {
  const uint32_t capture_flags =
      srf05_capture_flag(config.captureChannel) |
      srf05_overcapture_flag(config.captureChannel);

  capture_state = SRF05_CAPTURE_RISING;
  rising_capture = 0u;
  timer_overflows = 0u;
  pulse_ticks = 0u;
  capture_complete_ms = 0u;

  __HAL_TIM_SET_CAPTUREPOLARITY(config.timer, config.captureChannel,
                                TIM_INPUTCHANNELPOLARITY_RISING);
  if (capture_flags != 0u) {
    /* An overrun from the previous measurement must not poison the next one. */
    __HAL_TIM_CLEAR_FLAG(config.timer, capture_flags);
  }
  __HAL_TIM_CLEAR_FLAG(config.timer, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(config.timer, TIM_IT_UPDATE);

  if (HAL_TIM_IC_Start_IT(config.timer, config.captureChannel) != HAL_OK) {
    __HAL_TIM_DISABLE_IT(config.timer, TIM_IT_UPDATE);
    capture_state = SRF05_CAPTURE_IDLE;
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef SRF05_Init(SRF05_Config_t new_config) {
  if (initialized) {
    SRF05_CancelMeasurement();
  }
  initialized = 0u;

  if (new_config.timer == NULL || new_config.timer->Instance == NULL ||
      new_config.triggerPort == NULL || new_config.triggerPin == 0u ||
      (new_config.triggerPin & (new_config.triggerPin - 1u)) != 0u ||
      !isfinite(new_config.timer_tick_us) ||
      new_config.timer_tick_us <= 0.0f ||
      !srf05_channel_is_valid(new_config.captureChannel)) {
    return HAL_ERROR;
  }

  config = new_config;
  capture_state = SRF05_CAPTURE_IDLE;
  has_triggered = 0u;
  last_trigger_ms = 0u;
  measurement_start_ms = 0u;
  capture_complete_ms = 0u;

  HAL_GPIO_WritePin(config.triggerPort, config.triggerPin, GPIO_PIN_RESET);
  __HAL_TIM_SET_CAPTUREPOLARITY(config.timer, config.captureChannel,
                                TIM_INPUTCHANNELPOLARITY_RISING);

  initialized = 1u;
  return HAL_OK;
}

HAL_StatusTypeDef SRF05_StartMeasurement(void) {
  if (!initialized) {
    return HAL_ERROR;
  }

  if (capture_state != SRF05_CAPTURE_IDLE) {
    return HAL_BUSY;
  }

  const uint32_t now = HAL_GetTick();
  if (has_triggered &&
      (now - last_trigger_ms) <= SRF05_MIN_TRIGGER_PERIOD_MS) {
    return HAL_BUSY;
  }

  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  measurement_start_ms = HAL_GetTick();
  if (srf05_start_capture() != HAL_OK) {
    if (interrupt_mask == 0u) {
      __enable_irq();
    }
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(config.triggerPort, config.triggerPin, GPIO_PIN_SET);
  srf05_delay_us(SRF05_TRIGGER_PULSE_US);
  HAL_GPIO_WritePin(config.triggerPort, config.triggerPin, GPIO_PIN_RESET);

  /* Include the pulse itself in the quiet time. The extra tick in the spacing
   * check guarantees at least 50 ms despite HAL_GetTick's 1 ms quantization. */
  last_trigger_ms = HAL_GetTick();
  has_triggered = 1u;
  if (interrupt_mask == 0u) {
    __enable_irq();
  }

  return HAL_OK;
}

HAL_StatusTypeDef SRF05_ReadMeasurement(float *distance_cm,
                                        uint32_t *sample_time_ms) {
  if (!initialized || distance_cm == NULL || sample_time_ms == NULL) {
    return HAL_ERROR;
  }

  const uint32_t timeout_ms =
      (config.timeout_ms != 0u) ? config.timeout_ms : SRF05_DEFAULT_TIMEOUT_MS;
  if (capture_state == SRF05_CAPTURE_IDLE) {
    return HAL_ERROR;
  }

  if (capture_state != SRF05_CAPTURE_COMPLETE) {
    if ((capture_state == SRF05_CAPTURE_RISING ||
         capture_state == SRF05_CAPTURE_FALLING) &&
        (HAL_GetTick() - measurement_start_ms) >= timeout_ms) {
      const uint32_t interrupt_mask = __get_PRIMASK();
      __disable_irq();
      if (capture_state == SRF05_CAPTURE_COMPLETE) {
        if (interrupt_mask == 0u) {
          __enable_irq();
        }
        goto measurement_complete;
      }
      srf05_stop_capture();
      capture_state = SRF05_CAPTURE_IDLE;
      if (interrupt_mask == 0u) {
        __enable_irq();
      }
      return HAL_TIMEOUT;
    }
    return HAL_BUSY;
  }

measurement_complete:
  ;
  const uint64_t measured_ticks = pulse_ticks;
  const uint32_t completed_ms = capture_complete_ms;
  capture_state = SRF05_CAPTURE_IDLE;

  /* Completion may have happened after the deadline while the foreground
   * caller was busy. Judge the echo's completion time, not the read time. */
  if ((completed_ms - measurement_start_ms) >= timeout_ms) {
    return HAL_TIMEOUT;
  }

  if (measured_ticks == 0u) {
    return HAL_ERROR;
  }

  const float echo_time_us = (float)measured_ticks * config.timer_tick_us;
  const float measured_distance_cm = echo_time_us / SRF05_US_PER_CM;

  if (!isfinite(measured_distance_cm) ||
      measured_distance_cm < SRF05_MIN_VALID_DISTANCE_CM ||
      measured_distance_cm > SRF05_MAX_VALID_DISTANCE_CM) {
    return HAL_ERROR;
  }

  *distance_cm = measured_distance_cm;
  *sample_time_ms = completed_ms;
  return HAL_OK;
}

void SRF05_CancelMeasurement(void) {
  if (!initialized) {
    return;
  }

  const uint32_t interrupt_mask = __get_PRIMASK();
  __disable_irq();
  if (capture_state == SRF05_CAPTURE_RISING ||
      capture_state == SRF05_CAPTURE_FALLING) {
    srf05_stop_capture();
  }
  capture_state = SRF05_CAPTURE_IDLE;
  if (interrupt_mask == 0u) {
    __enable_irq();
  }
}

HAL_StatusTypeDef SRF05_MeasureDistance(float *distance_cm) {
  if (distance_cm == NULL || __get_IPSR() != 0u ||
      __get_PRIMASK() != 0u || __get_BASEPRI() != 0u ||
      __get_FAULTMASK() != 0u) {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = SRF05_StartMeasurement();
  if (status != HAL_OK) {
    return status;
  }

  uint32_t sample_time_ms;
  do {
    status = SRF05_ReadMeasurement(distance_cm, &sample_time_ms);
  } while (status == HAL_BUSY);

  return status;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (!initialized || htim != config.timer ||
      !srf05_is_active_channel(htim->Channel, config.captureChannel) ||
      (capture_state != SRF05_CAPTURE_RISING &&
       capture_state != SRF05_CAPTURE_FALLING)) {
    return;
  }

  /* Freeze only this input while validating CCR and changing edge polarity.
   * Do not stop/reset the shared counter or disturb its motor PWM channel. */
  TIM_CCxChannelCmd(htim->Instance, config.captureChannel, TIM_CCx_DISABLE);

  const uint32_t capture_flags = srf05_capture_flag(config.captureChannel) |
                                 srf05_overcapture_flag(config.captureChannel);
  const uint32_t pending_capture = __HAL_TIM_GET_FLAG(htim, capture_flags);
  const uint32_t captured =
      HAL_TIM_ReadCapturedValue(htim, config.captureChannel);
  /* HAL clears CCxIF before this callback. A new CCxIF means CCR was replaced
   * after that clear; in that case hardware need not set CCxOF. */
  if (pending_capture != RESET || __HAL_TIM_GET_FLAG(htim,
                        srf05_overcapture_flag(config.captureChannel)) != RESET) {
    srf05_stop_capture();
    pulse_ticks = 0u;
    capture_complete_ms = HAL_GetTick();
    capture_state = SRF05_CAPTURE_COMPLETE;
    return;
  }
  const uint64_t captured_ticks = srf05_capture_ticks(captured);

  if (capture_state == SRF05_CAPTURE_RISING) {
    rising_capture = captured_ticks;
    capture_state = SRF05_CAPTURE_FALLING;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, config.captureChannel,
                                  TIM_INPUTCHANNELPOLARITY_FALLING);
    TIM_CCxChannelCmd(htim->Instance, config.captureChannel, TIM_CCx_ENABLE);
    return;
  }

  if (capture_state == SRF05_CAPTURE_FALLING) {
    srf05_stop_capture();
    pulse_ticks = captured_ticks - rising_capture;
    capture_complete_ms = HAL_GetTick();
    capture_state = SRF05_CAPTURE_COMPLETE;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (!initialized || htim != config.timer ||
      (capture_state != SRF05_CAPTURE_RISING &&
       capture_state != SRF05_CAPTURE_FALLING)) {
    return;
  }

  timer_overflows++;
}
