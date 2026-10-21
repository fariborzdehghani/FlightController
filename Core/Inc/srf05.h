#ifndef SRF05_H
#define SRF05_H

#include "stm32f4xx_hal.h"

/* Measurements below this distance are not accepted. */
#define SRF05_MIN_VALID_DISTANCE_CM 8.0f

/* Measurements above the practical SRF05 range are not accepted. */
#define SRF05_MAX_VALID_DISTANCE_CM 400.0f

typedef struct {
  /* Up-counting, direct input capture, no input prescaling. The driver owns
   * this channel and the timer update interrupt; other PWM channels may run.
   * Keep ARR/PSC fixed while measuring and set timer_tick_us accordingly. */
  TIM_HandleTypeDef *timer;
  uint32_t captureChannel;
  GPIO_TypeDef *triggerPort;
  uint16_t triggerPin;
  uint32_t timeout_ms;
  float timer_tick_us;
} SRF05_Config_t;

HAL_StatusTypeDef SRF05_Init(SRF05_Config_t config);

/* One foreground caller must own init/start/read/cancel. Poll reads until a
 * result, or cancel before starting again. Start masks interrupts for the
 * 10 us trigger and capture setup; it returns HAL_BUSY during the quiet time
 * (at least 50 ms, guarded by 51 HAL ticks). Service capture IRQs within the
 * shortest accepted echo (464 us at 8 cm) and updates within one timer period.
 * Both HAL timer callbacks are implemented by this module. */
HAL_StatusTypeDef SRF05_StartMeasurement(void);
/* HAL_TIMEOUT also rejects pulses completed at/after the measurement deadline.
 * HAL_ERROR rejects capture overruns and out-of-range pulses. Output arguments
 * are written only on HAL_OK; sample_time_ms is the capture callback timestamp. */
HAL_StatusTypeDef SRF05_ReadMeasurement(float *distance_cm,
                                        uint32_t *sample_time_ms);
void SRF05_CancelMeasurement(void);

/* Blocking compatibility API. Requires main/thread context with interrupts
 * enabled, an advancing HAL tick, and the timer IRQ enabled in the NVIC. */
HAL_StatusTypeDef SRF05_MeasureDistance(float *distance_cm);

#endif /* SRF05_H */
