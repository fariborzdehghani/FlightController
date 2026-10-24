#include "MPU6050.h"
#include "SensorConfig.h"
#include "Tools.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static I2C_HandleTypeDef bus;
static uint32_t now_ms;
static uint8_t registers[128], raw[14];
static HAL_StatusTypeDef read_status = HAL_OK;
static HAL_StatusTypeDef config_read_status = HAL_OK;
static uint16_t ignored_write_reg = UINT16_MAX;
static char error_messages[8][96];
static unsigned error_count;

uint32_t HAL_GetTick(void) { return now_ms; }
void HAL_Delay(uint32_t delay) { now_ms += delay; }
bool LogInformation(int code, const char *message)
{
  (void)code;
  (void)message;
  return true;
}

bool LogError(int code, const char *message)
{
  assert(code == 2003);
  assert(error_count < sizeof(error_messages) / sizeof(error_messages[0]));
  assert(strlen(message) < sizeof(error_messages[0]));
  strcpy(error_messages[error_count++], message);
  return true;
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *i2c,
                                      uint16_t address, uint32_t trials,
                                      uint32_t timeout)
{
  assert(i2c == &bus && address == MPU6050_ADDR);
  (void)trials;
  (void)timeout;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c, uint16_t address,
                                 uint16_t reg, uint16_t address_size,
                                 uint8_t *data, uint16_t length,
                                 uint32_t timeout)
{
  assert(i2c == &bus && address == MPU6050_ADDR);
  assert(address_size == I2C_MEMADD_SIZE_8BIT);
  (void)timeout;
  if (read_status != HAL_OK) return read_status;
  if (reg == WHO_AM_I_REG)
  {
    assert(length == 1);
    data[0] = 0x68;
  }
  else if (reg == SMPLRT_DIV_REG)
  {
    assert(length == 4);
    if (config_read_status != HAL_OK) return config_read_status;
    memcpy(data, registers + reg, length);
  }
  else
  {
    assert(reg == ACCEL_XOUT_H_REG && length == sizeof(raw));
    memcpy(data, raw, length);
  }
  return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c, uint16_t address,
                                  uint16_t reg, uint16_t address_size,
                                  uint8_t *data, uint16_t length,
                                  uint32_t timeout)
{
  assert(i2c == &bus && address == MPU6050_ADDR);
  assert(address_size == I2C_MEMADD_SIZE_8BIT && length == 1);
  assert(reg < sizeof(registers));
  (void)timeout;
  if (reg == PWR_MGMT_1_REG && data[0] == 0x80)
  {
    memset(registers, 0, sizeof(registers));
    registers[PWR_MGMT_1_REG] = 0x40; /* Reset leaves the module asleep. */
    return HAL_OK;
  }
  /* Reproduce the attached module: configuration writes ACK while asleep
   * but do not take effect. Wake-up must precede them in production code. */
  if (reg != PWR_MGMT_1_REG && (registers[PWR_MGMT_1_REG] & 0x40))
    return HAL_OK;
  if (reg == ignored_write_reg) return HAL_OK;
  registers[reg] = data[0];
  return HAL_OK;
}

static void close_to(double actual, double expected, double tolerance)
{
  if (!(fabs(actual - expected) <= tolerance))
    fprintf(stderr, "actual %.12f, expected %.12f (+/- %.12f)\n",
            actual, expected, tolerance);
  assert(fabs(actual - expected) <= tolerance);
}

static double wrap(double angle)
{
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

static void set_word(unsigned offset, double value)
{
  assert(value >= -32768.0 && value <= 32767.0);
  const uint16_t encoded = (uint16_t)(int16_t)lround(value);
  raw[offset] = (uint8_t)(encoded >> 8);
  raw[offset + 1] = (uint8_t)encoded;
}

/* Angles use the driver's MPU frame, before aircraft roll/pitch remapping. */
static void set_sample(double angle_x, double angle_y, double norm_g,
                       double gx, double gy)
{
  const double x = angle_x / RAD_TO_DEG, y = angle_y / RAD_TO_DEG;
  memset(raw, 0, sizeof(raw));
  /* The simulated +/-8 g device emits 4096 counts per g. Keep this value
   * independent of the driver's conversion so a scaling error is detected. */
  set_word(0, -sin(y) * cos(x) * norm_g * 4096.0);
  set_word(2, sin(x) * norm_g * 4096.0);
  set_word(4, cos(y) * cos(x) * norm_g * 4096.0);
  set_word(8, gx * 131.0);
  set_word(10, gy * 131.0);
}

static MPU6050_t step(void)
{
  MPU6050_t result = {0};
  now_ms += 15;
  assert(MPU6050_ReadAll(&result) == HAL_OK);
  const AttitudeTrace_t *t = &result.attitude_trace;
  assert(t->valid && t->timestamp_ms == now_ms && t->dt_ms == 15.0f);
  close_to(t->accel_x_g, result.Ax, 1e-7);
  close_to(t->accel_y_g, result.Ay, 1e-7);
  close_to(t->accel_z_g, result.Az, 1e-7);
  close_to(t->roll.predicted_deg,
           t->roll.previous_deg + (result.Gy - t->roll.bias_dps) * 0.015, 3e-5);
  close_to(t->pitch.predicted_deg,
           t->pitch.previous_deg + (result.Gx - t->pitch.bias_dps) * 0.015, 3e-5);
  close_to(wrap((double)t->roll.predicted_deg + t->roll.correction_deg - result.KalmanAngleY), 0, 5e-5);
  close_to(wrap((double)t->pitch.predicted_deg + t->pitch.correction_deg - result.KalmanAngleX), 0, 5e-5);
  return result;
}

static void start(double angle_x, double angle_y)
{
  memset(registers, 0, sizeof(registers));
  read_status = HAL_OK;
  error_count = 0;
  set_sample(angle_x, angle_y, 1.0, 0.0, 0.0);
  assert(MPU6050_Init(&bus) == HAL_OK);
  assert(!MPU6050_IsReady());
  assert(MPU6050_Calibrate() == HAL_OK);
  assert(MPU6050_IsReady());
  assert(registers[SMPLRT_DIV_REG] == 7);
  assert(registers[ACCEL_CONFIG_REG] == 0x10); /* AFS_SEL=2: +/-8 g. */
  assert(registers[GYRO_CONFIG_REG] == 0); /* Still +/-250 deg/s. */
  const unsigned expected_dlpf = MPU6050_DLPF_HZ == 5 ? 6 :
      MPU6050_DLPF_HZ == 10 ? 5 : MPU6050_DLPF_HZ == 20 ? 4 : 3;
  assert(registers[CONFIG_REG] == expected_dlpf);
}

static void test_startup_range_and_calibration_failures(void)
{
  memset(registers, 0, sizeof(registers));
  error_count = 0;
  ignored_write_reg = ACCEL_CONFIG_REG;
  assert(MPU6050_Init(&bus) == HAL_ERROR);
  assert(!MPU6050_IsReady());
  assert(error_count == 1);
  assert(strstr(error_messages[0], "ACCEL_CONFIG mismatch: read=0x00 expected=0x10"));
  ignored_write_reg = SMPLRT_DIV_REG;
  error_count = 0;
  assert(MPU6050_Init(&bus) == HAL_ERROR);
  assert(!MPU6050_IsReady());
  assert(error_count == 1 && strstr(error_messages[0], "config mismatch: divider=0x00"));
  ignored_write_reg = CONFIG_REG;
  error_count = 0;
  assert(MPU6050_Init(&bus) == HAL_ERROR);
  assert(!MPU6050_IsReady());
  assert(error_count == 1 && strstr(error_messages[0], "filter=0x00"));
  ignored_write_reg = UINT16_MAX;

  error_count = 0;
  config_read_status = HAL_TIMEOUT;
  assert(MPU6050_Init(&bus) == HAL_TIMEOUT);
  assert(!MPU6050_IsReady());
  assert(error_count == 1 && strstr(error_messages[0], "configuration readback failed"));
  config_read_status = HAL_OK;

  /* A range/output mismatch must report the observed 4 g, not silently
   * recalibrate its scale or relax the stationary acceptance check. */
  start(0, 0);
  set_sample(0, 0, 4.0, 0, 0);
  assert(MPU6050_Calibrate() == HAL_TIMEOUT);
  assert(!MPU6050_IsReady());
  assert(error_count == 2);
  assert(strstr(error_messages[0], "norm=4.000..4.000 g gyro_peak=0.00 dps"));
  assert(strstr(error_messages[1], "accel=2400 gyro=0 longest=0/600 samples"));

  start(0, 0);
  set_sample(0, 0, 1.0, 4.0, 0);
  assert(MPU6050_Calibrate() == HAL_TIMEOUT);
  assert(!MPU6050_IsReady());
  assert(error_count == 2);
  assert(strstr(error_messages[0], "norm=1.000..1.000 g gyro_peak=4.00 dps"));
  assert(strstr(error_messages[1], "accel=0 gyro=2400 longest=0/600 samples"));
  start(0, 0); /* Successful retry restores readiness. */
}

static void test_acceleration_range_and_units(void)
{
  start(0.0, 0.0); /* A 4096-count stationary sample must calibrate as 1 g. */
  MPU6050_t result = step();
  close_to(result.Az, 1.0, 1e-12);
  close_to(result.Az_ms, 9.80665, 1e-9);

#if MPU6050_ACCEL_REJECTION_ENABLED
  /* Changing sensor scale must not widen the existing 0.8-1.2 g gate. */
  const double norms[] = {0.79, 0.81, 1.19, 1.21};
  for (unsigned i = 0; i < 4; ++i)
  {
    set_sample(0, 0, norms[i], 0, 0);
    result = step();
    const uint8_t expected = (i == 0 || i == 3) ?
        ATTITUDE_TRACE_MAGNITUDE_REJECTED : ATTITUDE_TRACE_CORRECTION_APPLIED;
    assert(result.attitude_trace.roll_flags == expected);
    assert(result.attitude_trace.pitch_flags == expected);
  }
#endif

  /* Peaks beyond the old +/-2 g range remain measurable and are rejected
   * as gravity observations by the unchanged filter magnitude gate. */
  const double peaks[] = {3.0, -4.0, 6.0};
  for (unsigned axis = 0; axis < 3; ++axis)
  {
    memset(raw, 0, sizeof(raw));
    set_word(2 * axis, peaks[axis] * 4096.0);
    result = step();
    const double acceleration[] = {result.Ax, result.Ay, result.Az};
    const double acceleration_ms[] = {result.Ax_ms, result.Ay_ms, result.Az_ms};
    close_to(acceleration[axis], peaks[axis], 1e-12);
    close_to(acceleration_ms[axis], peaks[axis] * 9.80665, 1e-9);
    assert(!(result.attitude_trace.timing_flags & ATTITUDE_TRACE_ACCEL_CLIPPED));
#if MPU6050_ACCEL_REJECTION_ENABLED
    assert(result.attitude_trace.roll_flags & ATTITUDE_TRACE_MAGNITUDE_REJECTED);
    assert(result.attitude_trace.pitch_flags & ATTITUDE_TRACE_MAGNITUDE_REJECTED);
    assert(result.attitude_trace.roll.correction_deg == 0);
    assert(result.attitude_trace.pitch.correction_deg == 0);
#endif
  }

  memset(raw, 0, sizeof(raw));
  set_word(0, -32768);
  set_word(4, 32767);
  result = step();
  close_to(result.Ax, -8.0, 1e-12);
  close_to(result.Az, 32767.0 / 4096.0, 1e-12);
  assert(result.attitude_trace.timing_flags & ATTITUDE_TRACE_ACCEL_CLIPPED);
}

static void test_normal_motion(void)
{
  start(12.0, -23.0);
  for (unsigned i = 0; i < 200; ++i)
  {
    const MPU6050_t result = step();
    close_to(result.KalmanAngleX, 12.0, 0.01);
    close_to(result.KalmanAngleY, -23.0, 0.01);
  }

  start(0.0, 0.0);
  Kalman_t expected_x = {.Q_angle = 0.001, .Q_bias = 0.003, .R_measure = 0.03};
  Kalman_t expected_y = expected_x;
  /* Good measurements produce the original Kalman result, including at
   * absolute angles larger than the innovation rejection threshold. */
  for (unsigned i = 1; i <= 200; ++i)
  {
    set_sample(i * 0.075, i * 0.15, 1.0, 5.0, 10.0);
    const MPU6050_t result = step();
    const double ax = result.Accel_X_RAW, ay = result.Accel_Y_RAW;
    const double az = result.Accel_Z_RAW;
    const double angle_x = atan2(ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG;
    const double angle_y = atan2(-ax, az) * RAD_TO_DEG;
    close_to(result.KalmanAngleX,
             Kalman_GetAngle(&expected_x, angle_x, 5.0, 0.015), 1e-9);
    close_to(result.KalmanAngleY,
             Kalman_GetAngle(&expected_y, angle_y, 10.0, 0.015), 1e-9);
  }
}

static void test_invalid_norm_and_recovery(void)
{
  start(0.0, 0.0);
  MPU6050_t result = {0};
  for (unsigned i = 0; i < 100; ++i)
  {
    /* Freefall, low magnitude and excessive acceleration all reject both
     * corrections. Gyro integration must continue without learning bias. */
    const double norm = i % 3 == 0 ? 0.0 : i % 3 == 1 ? 0.5 : 1.6;
    set_sample(-20.0, -40.0, norm, 10.0, 20.0);
    result = step();
#if MPU6050_ACCEL_REJECTION_ENABLED
    close_to(result.KalmanAngleX, (i + 1) * 0.15, 1e-9);
    close_to(result.KalmanAngleY, (i + 1) * 0.30, 1e-9);
#endif
  }
#if MPU6050_ACCEL_REJECTION_ENABLED
  set_sample(15.0, 30.0, 1.0, 0.0, 0.0);
  for (unsigned i = 0; i < 100; ++i)
  {
    result = step();
    close_to(result.KalmanAngleX, 15.0, 0.01);
    close_to(result.KalmanAngleY, 30.0, 0.01);
  }
#else
  /* Disabling the feature restores unrestricted accelerometer influence. */
  assert(fabs(result.KalmanAngleY - 30.0) > 10.0);
#endif
}

static void test_direction_spikes_and_recovery(void)
{
  start(0.0, 0.0);
  for (unsigned i = 0; i < 200; ++i) (void)step();
  set_sample(40.0, 60.0, 1.0, 0.0, 0.0);
  MPU6050_t result = {0};
  for (unsigned i = 0; i < 40; ++i) result = step();
#if MPU6050_ACCEL_REJECTION_ENABLED
  close_to(result.KalmanAngleX, 0.0, 1e-12);
  close_to(result.KalmanAngleY, 0.0, 1e-12);
  /* Plausible small corrections resume; the filter is not frozen after a
   * rejected burst, and propagated covariance still permits correction. */
  set_sample(2.0, -2.0, 1.0, 0.0, 0.0);
  for (unsigned i = 0; i < 100; ++i) result = step();
  assert(result.KalmanAngleX > 1.0 && result.KalmanAngleX < 3.0);
  assert(result.KalmanAngleY < -1.0 && result.KalmanAngleY > -3.0);
#else
  assert(result.KalmanAngleX > 10.0 && result.KalmanAngleY > 15.0);
#endif
}

static void test_stationary_recovery_after_large_error(void)
{
  /* Regression: a 300 ms disturbance used to leave a level, stationary
   * sensor at aircraft roll=36 / pitch=24 degrees indefinitely. */
  const double initial_x[] = {0.0, 12.0, 0.0, 0.0};
  const double initial_y[] = {0.0, -23.0, 179.0, -179.0};
  for (unsigned scenario = 0; scenario < 4; ++scenario)
  {
    start(initial_x[scenario], initial_y[scenario]);
    for (unsigned i = 0; i < 200; ++i) (void)step();
    set_sample(0.0, 0.0, 0.5, 80.0, 120.0);
    MPU6050_t result = {0};
    for (unsigned i = 0; i < 20; ++i) result = step();

    set_sample(initial_x[scenario], initial_y[scenario], 1.0, 0.0, 0.0);
#if MPU6050_ACCEL_REJECTION_ENABLED
    uint8_t observed_flags = 0;
#endif
    for (unsigned i = 0; i < 667; ++i)
    {
      const MPU6050_t previous = result;
      result = step();
#if MPU6050_ACCEL_REJECTION_ENABLED
      observed_flags |= result.attitude_trace.roll_flags | result.attitude_trace.pitch_flags;
      /* Recovery corrects at most 30 deg/s, including across +/-180. */
      assert(fabs(wrap(result.KalmanAngleX - previous.KalmanAngleX)) < 0.451);
      assert(fabs(wrap(result.KalmanAngleY - previous.KalmanAngleY)) < 0.451);
#else
      (void)previous;
#endif
    }
#if MPU6050_ACCEL_REJECTION_ENABLED
    assert(observed_flags & ATTITUDE_TRACE_RECOVERY_WAITING);
    assert(observed_flags & ATTITUDE_TRACE_RECOVERY_APPLIED);
    assert(observed_flags & ATTITUDE_TRACE_STATE_RESET);
#endif
    close_to(result.KalmanAngleX, initial_x[scenario], 0.01);
    close_to(wrap(result.KalmanAngleY - initial_y[scenario]), 0.0, 0.01);
  }
}

static void test_recovery_near_reported_angles(void)
{
  start(0.0, 0.0);
  for (unsigned i = 0; i < 200; ++i) (void)step();
  set_sample(0.0, 0.0, 0.5, 0.0, -120.0);
  MPU6050_t result = {0};
  for (unsigned i = 0; i < 81; ++i) result = step();
  set_sample(0.0, 0.0, 0.5, 120.0, 0.0);
  for (unsigned i = 0; i < 99; ++i) result = step();
#if MPU6050_ACCEL_REJECTION_ENABLED
  close_to(result.KalmanAngleY, -145.8, 0.001);
  close_to(result.KalmanAngleX, -178.2, 0.001);
#endif
  /* Include residual gyro rates comparable to the stationary log. */
  set_sample(0.0, 0.0, 1.0, -0.14, -0.13);
  for (unsigned i = 0; i < 1000; ++i) result = step();
  close_to(result.KalmanAngleX, 0.0, 0.1);
  close_to(result.KalmanAngleY, 0.0, 0.1);
}

#if MPU6050_ACCEL_REJECTION_ENABLED
static void test_recovery_requires_consistent_gravity(void)
{
  start(0.0, 0.0);
  set_sample(0.0, 0.0, 0.5, 80.0, 120.0);
  MPU6050_t before = {0};
  for (unsigned i = 0; i < 20; ++i) before = step();

  /* Repeated direction changes must not eventually open the angle gate. */
  for (unsigned i = 0; i < 400; ++i)
  {
    set_sample(i % 2 ? 0.0 : -20.0, i % 2 ? 0.0 : -40.0,
               1.0, 0.0, 0.0);
    const MPU6050_t result = step();
    close_to(result.KalmanAngleX, before.KalmanAngleX, 1e-9);
    close_to(result.KalmanAngleY, before.KalmanAngleY, 1e-9);
  }

  /* Good gravity for 600 ms followed by invalid magnitude must restart the
   * stability window. Repeated fragments cannot accumulate into recovery. */
  for (unsigned burst = 0; burst < 5; ++burst)
  {
    for (unsigned i = 0; i < 40; ++i)
    {
      set_sample(0.0, 0.0, 1.0, 0.0, 0.0);
      const MPU6050_t result = step();
      close_to(result.KalmanAngleX, before.KalmanAngleX, 1e-9);
      close_to(result.KalmanAngleY, before.KalmanAngleY, 1e-9);
    }
    set_sample(0.0, 0.0, 0.5, 0.0, 0.0);
    (void)step();
  }

  /* A gyro rate on the third axis must also restart the quiet window. */
  for (unsigned burst = 0; burst < 5; ++burst)
  {
    set_sample(0.0, 0.0, 1.0, 0.0, 0.0);
    for (unsigned i = 0; i < 40; ++i) (void)step();
    set_word(12, 10.0 * 131.0);
    const MPU6050_t result = step();
    close_to(result.KalmanAngleX, before.KalmanAngleX, 1e-9);
    close_to(result.KalmanAngleY, before.KalmanAngleY, 1e-9);
  }

  /* Once recovery starts, unstable directions must cancel it as well. */
  set_sample(0.0, 0.0, 1.0, 0.0, 0.0);
  for (unsigned i = 0; i < 75; ++i) before = step();
  assert(before.KalmanAngleX > 15.0 && before.KalmanAngleX < 24.0);
  assert(before.KalmanAngleY > 15.0 && before.KalmanAngleY < 36.0);
  for (unsigned i = 0; i < 100; ++i)
  {
    set_sample(i % 2 ? 0.0 : -20.0, i % 2 ? 0.0 : -40.0,
               1.0, 0.0, 0.0);
    const MPU6050_t result = step();
    close_to(result.KalmanAngleX, before.KalmanAngleX, 1e-9);
    close_to(result.KalmanAngleY, before.KalmanAngleY, 1e-9);
  }

  set_sample(0.0, 0.0, 1.0, 0.0, 0.0);
  MPU6050_t result = {0};
  for (unsigned i = 0; i < 667; ++i) result = step();
  close_to(result.KalmanAngleX, 0.0, 0.01);
  close_to(result.KalmanAngleY, 0.0, 0.01);
}

static void test_bias_is_bounded(void)
{
  start(0.0, 0.0);
  for (unsigned i = 0; i < 200; ++i) (void)step();
  /* Mismatched gyro/accel data can teach the Kalman filter a false bias. */
  set_sample(0.0, 0.0, 1.0, 0.0, 30.0);
  MPU6050_t before = {0};
  for (unsigned i = 0; i < 400; ++i) before = step();
  set_sample(0.0, 0.0, 0.5, 0.0, 0.0);
  MPU6050_t after = {0};
  for (unsigned i = 0; i < 100; ++i) after = step();
  /* With no accel correction, drift is bounded by 3 deg/s for 1.5 seconds. */
  assert(fabs(wrap(after.KalmanAngleY - before.KalmanAngleY)) <= 4.501);
  set_sample(0.0, 0.0, 1.0, 0.0, 0.0);
  for (unsigned i = 0; i < 1000; ++i) after = step();
  close_to(after.KalmanAngleY, 0.0, 0.01);
}
#endif

static void test_angle_wrap_and_reset_bypass(void)
{
  start(0.0, 179.0);
  for (unsigned i = 1; i <= 20; ++i)
  {
    const double angle = wrap(179.0 + i * 0.15);
    set_sample(0.0, angle, 1.0, 0.0, 10.0);
    const MPU6050_t result = step();
    close_to(wrap(result.KalmanAngleY - angle), 0.0, 0.01);
  }
  start(0.0, 110.0);
  const MPU6050_t before = step();
  set_sample(0.0, -110.0, 1.0, 0.0, 0.0);
  const MPU6050_t after = step();
#if MPU6050_ACCEL_REJECTION_ENABLED
  close_to(after.KalmanAngleY, before.KalmanAngleY, 1e-9);
#else
  (void)before;
  close_to(after.KalmanAngleY, -110.0, 0.01);
  assert(after.attitude_trace.roll_flags == ATTITUDE_TRACE_LEGACY_RESET);
#endif
}

static void test_read_errors_and_clock_wrap(void)
{
  now_ms = UINT32_MAX - 3405u; /* Startup/calibration advances 3400 ms. */
  start(0.0, 0.0);
  set_sample(0.0, 0.3, 1.0, 0.0, 20.0);
  MPU6050_t result = step();
  assert(now_ms < 15u);
  close_to(result.KalmanAngleY, 0.3, 0.01);
  read_status = HAL_TIMEOUT;
  const MPU6050_t before = result;
  assert(MPU6050_ReadAll(&result) == HAL_TIMEOUT);
  assert(memcmp(&result, &before, sizeof(result)) == 0);
  read_status = HAL_OK;
}

static void test_trace_flags_and_timing(void)
{
  start(0.0, 0.0);
  MPU6050_t first = step();
  assert(first.attitude_trace.sequence == 1);
  assert(first.attitude_trace.roll_flags == ATTITUDE_TRACE_CORRECTION_APPLIED);
  set_sample(0, 0, 0.5, 10, -20);
  MPU6050_t rejected = step();
  assert(rejected.attitude_trace.sequence == 2);
#if MPU6050_ACCEL_REJECTION_ENABLED
  assert(rejected.attitude_trace.roll_flags & ATTITUDE_TRACE_MAGNITUDE_REJECTED);
  assert(!(rejected.attitude_trace.roll_flags & ATTITUDE_TRACE_CORRECTION_APPLIED));
  assert(rejected.attitude_trace.roll.correction_deg == 0);
#else
  assert(rejected.attitude_trace.roll_flags == ATTITUDE_TRACE_CORRECTION_APPLIED);
#endif
  MPU6050_t result = {0};
  assert(MPU6050_ReadAll(&result) == HAL_OK); /* Same millisecond. */
  assert(result.attitude_trace.dt_ms == 1);
  assert(result.attitude_trace.timing_flags & ATTITUDE_TRACE_DT_ZERO_CLAMPED);
  now_ms += 200;
  assert(MPU6050_ReadAll(&result) == HAL_OK);
  assert(result.attitude_trace.dt_ms == 100);
  assert(result.attitude_trace.timing_flags & ATTITUDE_TRACE_DT_HIGH_CLAMPED);
  set_word(0, -32768); set_word(10, 32767);
  result = step();
  assert((result.attitude_trace.timing_flags & (ATTITUDE_TRACE_ACCEL_CLIPPED | ATTITUDE_TRACE_GYRO_CLIPPED)) ==
         (ATTITUDE_TRACE_ACCEL_CLIPPED | ATTITUDE_TRACE_GYRO_CLIPPED));
}

int main(void)
{
  test_startup_range_and_calibration_failures();
  test_acceleration_range_and_units();
  test_normal_motion();
  test_invalid_norm_and_recovery();
  test_direction_spikes_and_recovery();
  test_stationary_recovery_after_large_error();
  test_recovery_near_reported_angles();
#if MPU6050_ACCEL_REJECTION_ENABLED
  test_recovery_requires_consistent_gravity();
  test_bias_is_bounded();
#endif
  test_angle_wrap_and_reset_bypass();
  test_read_errors_and_clock_wrap();
  test_trace_flags_and_timing();
  printf("MPU6050 tests passed (rejection=%u, cutoff=%u Hz)\n",
         MPU6050_ACCEL_REJECTION_ENABLED, MPU6050_DLPF_HZ);
  return 0;
}
