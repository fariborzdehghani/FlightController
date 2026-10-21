#include "srf05.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

uint32_t SystemCoreClock = 168000000;
static uint32_t now_ms;
static TIM_HandleTypeDef timer;
static TIM_TypeDef timer_registers;
static GPIO_TypeDef gpio;
static uint32_t channel;
static uint32_t inject_overcapture_on_read;
static HAL_StatusTypeDef start_status;
static float distance_cm;
static uint32_t sample_ms;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

uint32_t HAL_GetTick(void) { return now_ms; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state)
{
  (void)pin;
  port->pin_state = state;
}
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t ch)
{
  if (start_status != HAL_OK) return start_status;
  TIM_CCxChannelCmd(htim->Instance, ch, TIM_CCx_ENABLE);
  htim->counter_enabled = 1u;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t ch)
{
  /* Model the production HAL: stopping capture leaves other channels running. */
  TIM_CCxChannelCmd(htim->Instance, ch, TIM_CCx_DISABLE);
  if (!htim->Instance->CCER) htim->counter_enabled = 0u;
  return HAL_OK;
}
void TIM_CCxChannelCmd(TIM_TypeDef *instance, uint32_t ch, uint32_t state)
{
  if (state == TIM_CCx_ENABLE) instance->CCER |= 1u << ch;
  else instance->CCER &= ~(1u << ch);
}
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t ch)
{
  htim->flags &= ~(TIM_FLAG_CC1 << (ch / 4u));
  htim->flags |= inject_overcapture_on_read;
  inject_overcapture_on_read = 0u;
  return htim->captured;
}

static void init(uint32_t ch, uint32_t timeout_ms, uint32_t tick)
{
  SRF05_CancelMeasurement();
  timer_registers = (TIM_TypeDef){.CCER = 1};
  timer = (TIM_HandleTypeDef){.Instance = &timer_registers, .arr = 19999,
                             .counter_enabled = 1}; /* CH1 motor PWM */
  channel = ch;
  now_ms = tick;
  inject_overcapture_on_read = 0u;
  start_status = HAL_OK;
  distance_cm = -1;
  sample_ms = UINT32_MAX;
  SRF05_Config_t cfg = {.timer = &timer, .captureChannel = ch,
      .triggerPort = &gpio, .triggerPin = 1, .timeout_ms = timeout_ms,
      .timer_tick_us = 1};
  assert(SRF05_Init(cfg) == HAL_OK);
}

static void begin(void)
{
  assert(SRF05_StartMeasurement() == HAL_OK);
  assert(gpio.pin_state == GPIO_PIN_RESET);
}

static void delayed_edge(uint32_t count, uint32_t counter, uint32_t tick)
{
  timer.Channel = HAL_TIM_ACTIVE_CHANNEL_1 << (channel / 4u);
  timer.captured = count;
  timer.counter = counter;
  now_ms = tick;
  HAL_TIM_IC_CaptureCallback(&timer);
}

static void edge(uint32_t count, uint32_t tick)
{
  delayed_edge(count, count, tick);
}

static void overflow(void)
{
  timer.flags &= ~TIM_FLAG_UPDATE; /* HAL clears UIF before the callback. */
  HAL_TIM_PeriodElapsedCallback(&timer);
}

static void expect(HAL_StatusTypeDef status)
{
  assert(SRF05_ReadMeasurement(&distance_cm, &sample_ms) == status);
  if (status != HAL_OK)
  {
    assert(distance_cm == -1 && sample_ms == UINT32_MAX);
  }
}

static void motor_still_running(void)
{
  assert(timer.Instance->CCER == 1u);
  assert(timer.counter_enabled);
  assert(!(timer.interrupts & TIM_IT_UPDATE));
  assert(timer.arr == 19999);
}

static void test_timely_and_late_completion(void)
{
  init(TIM_CHANNEL_3, 60, 0);
  begin();
  edge(1000, 1);
  edge(3900, 4);
  now_ms = 100; /* Foreground delay does not invalidate an on-time capture. */
  expect(HAL_OK);
  assert(fabsf(distance_cm - 50) < 0.001f && sample_ms == 4);
  motor_still_running();

  for (uint32_t end = 59; end <= 61; end++)
  {
    init(TIM_CHANNEL_3, 60, 0);
    begin();
    edge(1000, end - 3);
    edge(3900, end);
    expect(end < 60 ? HAL_OK : HAL_TIMEOUT);
    motor_still_running();
  }
  init(TIM_CHANNEL_3, 0, 0); /* Zero selects the default 60 ms deadline. */
  begin();
  edge(5000, 65);
  edge(7900, 68);
  now_ms = 70;
  expect(HAL_TIMEOUT); /* Reproduced bug: previously returned 50 cm. */
  motor_still_running();
}

static void test_incomplete_timeout_and_recovery(void)
{
  for (unsigned rising_seen = 0; rising_seen < 2; rising_seen++)
  {
    init(TIM_CHANNEL_3, 60, 0);
    begin();
    if (rising_seen) edge(1000, 1);
    now_ms = 59;
    expect(HAL_BUSY);
    now_ms = 60;
    expect(HAL_TIMEOUT);
    motor_still_running();
    begin();
    edge(1000, 61);
    edge(3900, 64);
    expect(HAL_OK);
  }
}

static void test_overcapture_on_each_edge_and_channel(void)
{
  for (uint32_t ch = TIM_CHANNEL_1; ch <= TIM_CHANNEL_4; ch += 4)
  {
    for (unsigned falling = 0; falling < 2; falling++)
    {
      init(ch, 60, 0);
      begin();
      if (falling) edge(1000, 1);
      timer.flags |= TIM_FLAG_CC1OF << (ch / 4u);
      edge(falling ? 3900 : 1000, 4);
      expect(HAL_ERROR);
      if (ch != TIM_CHANNEL_1) motor_still_running();
      now_ms = 51;
      begin(); /* Starting again clears this channel's old overcapture flag. */
      assert(!(timer.flags & (TIM_FLAG_CC1OF << (ch / 4u))));
      edge(1000, 51);
      edge(3900, 54);
      expect(HAL_OK);
    }
  }
  init(TIM_CHANNEL_3, 60, 0);
  timer.flags = TIM_FLAG_CC3OF | TIM_FLAG_CC3 | TIM_FLAG_CC2OF;
  begin();
  assert(timer.flags == TIM_FLAG_CC2OF); /* Other channels' flags are preserved. */
  edge(1000, 1);
  inject_overcapture_on_read = TIM_FLAG_CC3OF;
  edge(3900, 4);
  expect(HAL_ERROR);
  motor_still_running();
}

static void test_timer_rollover(void)
{
  init(TIM_CHANNEL_3, 60, 0);
  begin();
  edge(19000, 1);
  overflow();
  edge(1900, 4);
  expect(HAL_OK);
  assert(fabsf(distance_cm - 50) < 0.001f);

  init(TIM_CHANNEL_3, 60, 0);
  begin();
  edge(19000, 1);
  timer.flags |= TIM_FLAG_UPDATE; /* Falling capture and overflow pending together. */
  edge(1900, 4);
  overflow();
  expect(HAL_OK);
  assert(fabsf(distance_cm - 50) < 0.001f);

  init(TIM_CHANNEL_3, 60, 0);
  begin();
  timer.flags |= TIM_FLAG_UPDATE; /* Overflow precedes the rising edge. */
  edge(1000, 1);
  overflow();
  edge(3900, 4);
  expect(HAL_OK);
  assert(fabsf(distance_cm - 50) < 0.001f);

  init(TIM_CHANNEL_3, 60, 0);
  begin();
  edge(19000, 1);
  overflow();
  overflow();
  edge(2200, 25); /* A valid 4 m pulse spans two counter wraps at this phase. */
  expect(HAL_OK);
  assert(fabsf(distance_cm - 400) < 0.001f);
  motor_still_running();
}

static void test_tick_wrap_custom_timeout_and_trigger_spacing(void)
{
  const uint32_t start = UINT32_MAX - 10;
  init(TIM_CHANNEL_3, 20, start);
  begin();
  assert(SRF05_StartMeasurement() == HAL_BUSY);
  edge(1000, start + 12u);
  edge(3900, start + 15u);
  expect(HAL_OK);
  assert(sample_ms == start + 15u);
  now_ms = start + 49u;
  assert(SRF05_StartMeasurement() == HAL_BUSY);
  now_ms = start + 50u;
  assert(SRF05_StartMeasurement() == HAL_BUSY);
  now_ms = start + 51u;
  begin();
  SRF05_CancelMeasurement();
  motor_still_running();

  init(TIM_CHANNEL_3, 20, start);
  begin();
  edge(1000, start + 18u);
  edge(3900, start + 21u);
  expect(HAL_TIMEOUT);
}

static void test_capture_arriving_during_update_irq(void)
{
  /* HAL snapshots SR on entry. An edge arriving after that snapshot is
   * dispatched in the next IRQ, after the current update callback. */
  init(TIM_CHANNEL_3, 60, 0);
  begin();
  edge(17000, 1);
  overflow();
  delayed_edge(19900, 100, 4); /* Falling edge was before the serviced wrap. */
  expect(HAL_OK);
  assert(fabsf(distance_cm - 50) < 0.001f);
  motor_still_running();

  init(TIM_CHANNEL_3, 60, 0);
  begin();
  overflow();
  delayed_edge(19900, 100, 1); /* Same ordering for the rising edge. */
  edge(2800, 4);
  expect(HAL_OK);
  assert(fabsf(distance_cm - 50) < 0.001f);
  motor_still_running();
}

static void test_pending_wrap_after_edge_and_replaced_capture(void)
{
  init(TIM_CHANNEL_3, 60, 0);
  begin();
  edge(17000, 1);
  timer.flags |= TIM_FLAG_UPDATE;
  delayed_edge(19900, 100, 4);
  expect(HAL_OK);
  assert(fabsf(distance_cm - 50) < 0.001f);
  motor_still_running();

  init(TIM_CHANNEL_3, 60, 0);
  begin();
  timer.flags |= TIM_FLAG_UPDATE;
  delayed_edge(19900, 100, 1);
  overflow();
  edge(2800, 4);
  expect(HAL_OK);
  assert(fabsf(distance_cm - 50) < 0.001f);

  for (unsigned falling = 0; falling < 2; falling++)
  {
    init(TIM_CHANNEL_3, 60, 0);
    begin();
    if (falling) edge(1000, 1);
    /* A new edge after HAL clears CC3IF can overwrite CCR without CC3OF. */
    timer.flags |= TIM_FLAG_CC3;
    edge(3900, 4);
    expect(HAL_ERROR);
    motor_still_running();
  }
}

static void test_full_width_timer_and_invalid_configuration(void)
{
  init(TIM_CHANNEL_3, 60, 0);
  timer.arr = UINT32_MAX;
  begin();
  edge(UINT32_MAX - 999u, 1);
  timer.flags |= TIM_FLAG_UPDATE;
  edge(1900, 4);
  expect(HAL_OK);
  assert(fabsf(distance_cm - 50) < 0.001f);

  SRF05_Config_t cfg = {.timer = &timer, .captureChannel = TIM_CHANNEL_3,
      .triggerPort = &gpio, .triggerPin = 1, .timer_tick_us = 1};
  cfg.timer = NULL;
  assert(SRF05_Init(cfg) == HAL_ERROR);
  assert(SRF05_StartMeasurement() == HAL_ERROR);
  cfg.timer = &timer;
  timer.Instance = NULL;
  assert(SRF05_Init(cfg) == HAL_ERROR);
  timer.Instance = &timer_registers;
  cfg.triggerPort = NULL;
  assert(SRF05_Init(cfg) == HAL_ERROR);
  cfg.triggerPort = &gpio;
  cfg.triggerPin = 0;
  assert(SRF05_Init(cfg) == HAL_ERROR);
  cfg.triggerPin = 3;
  assert(SRF05_Init(cfg) == HAL_ERROR);
  cfg.triggerPin = 1;
  cfg.timer_tick_us = NAN;
  assert(SRF05_Init(cfg) == HAL_ERROR);
  cfg.timer_tick_us = INFINITY;
  assert(SRF05_Init(cfg) == HAL_ERROR);
  cfg.timer_tick_us = 0;
  assert(SRF05_Init(cfg) == HAL_ERROR);
  cfg.timer_tick_us = 1;
  cfg.captureChannel = 123;
  assert(SRF05_Init(cfg) == HAL_ERROR);
  cfg.captureChannel = TIM_CHANNEL_3;
  assert(SRF05_Init(cfg) == HAL_OK);
}

static void timed_edge(uint32_t captured_ticks, uint32_t delay_ticks,
                       uint32_t *serviced_wraps, unsigned pending_update)
{
  const uint32_t service_ticks = captured_ticks + delay_ticks;
  const uint32_t wraps = service_ticks / 20000u;
  while (*serviced_wraps < wraps)
  {
    (*serviced_wraps)++;
    if (pending_update && *serviced_wraps == wraps)
      timer.flags |= TIM_FLAG_UPDATE;
    else
      overflow();
  }
  delayed_edge(captured_ticks % 20000u, service_ticks % 20000u,
               service_ticks / 1000u);
  if (timer.flags & TIM_FLAG_UPDATE) overflow();
}

static void test_timer_phase_sweep(void)
{
  const uint32_t widths[] = {464, 2900, 11600, 23200};
  for (uint32_t phase = 0; phase < 20000; phase += 97)
  {
    for (unsigned width = 0; width < sizeof(widths) / sizeof(widths[0]); width++)
    {
      for (unsigned ordering = 0; ordering < 4; ordering++)
      {
        init(TIM_CHANNEL_3, 60, 0);
        timer.counter = phase;
        begin();
        uint32_t serviced_wraps = 0;
        const uint32_t rise = phase + 700u;
        const uint32_t delay = (ordering & 1u) ? 120u : 0u;
        timed_edge(rise, delay, &serviced_wraps, ordering & 2u);
        timed_edge(rise + widths[width], delay, &serviced_wraps, ordering & 2u);
        expect(HAL_OK);
        assert(fabsf(distance_cm - (float)widths[width] / 58.0f) < 0.001f);
        motor_still_running();
      }
    }
  }
}

static void test_cancel_range_and_start_failure(void)
{
  init(TIM_CHANNEL_3, 60, 0);
  begin();
  edge(1000, 1);
  SRF05_CancelMeasurement();
  edge(3900, 4); /* A pending callback cannot revive a cancelled sample. */
  expect(HAL_ERROR);
  motor_still_running();

  init(TIM_CHANNEL_3, 60, 0);
  begin();
  edge(1000, 1);
  edge(1400, 2); /* Below the configured 8 cm minimum. */
  expect(HAL_ERROR);
  motor_still_running();

  init(TIM_CHANNEL_3, 60, 0);
  begin();
  edge(1000, 1);
  overflow();
  edge(5000, 25); /* Above the configured 4 m maximum. */
  expect(HAL_ERROR);

  init(TIM_CHANNEL_3, 60, 0);
  start_status = HAL_ERROR;
  assert(SRF05_StartMeasurement() == HAL_ERROR);
  motor_still_running();
  start_status = HAL_OK;
  begin();
}

int main(void)
{
  test_timely_and_late_completion();
  test_incomplete_timeout_and_recovery();
  test_overcapture_on_each_edge_and_channel();
  test_timer_rollover();
  test_tick_wrap_custom_timeout_and_trigger_spacing();
  test_capture_arriving_during_update_irq();
  test_pending_wrap_after_edge_and_replaced_capture();
  test_full_width_timer_and_invalid_configuration();
  test_timer_phase_sweep();
  test_cancel_range_and_start_failure();
  puts("PASS: 10 SRF05 regression groups (including 3312 timer-phase cases)");
  return 0;
}
