// Read data from consecutive registers of an I2C device
#include "MPU6050.h"
#include "SensorConfig.h"
#include "Tools.h"
#include <math.h>
#include <stdio.h>

#define MPU6050_GYRO_SCALE_LSB_PER_DPS 131.0
#define MPU6050_CALIBRATION_DISCARD_SAMPLES 20U
#define MPU6050_CALIBRATION_SAMPLE_DELAY_MS 5U
#define MPU6050_CALIBRATION_DURATION_MS 3000U
#define MPU6050_CALIBRATION_REQUIRED_SAMPLES                              \
  ((MPU6050_CALIBRATION_DURATION_MS +                                    \
    MPU6050_CALIBRATION_SAMPLE_DELAY_MS - 1U) /                          \
   MPU6050_CALIBRATION_SAMPLE_DELAY_MS)
#define MPU6050_CALIBRATION_MAX_ATTEMPTS                                 \
  (MPU6050_CALIBRATION_REQUIRED_SAMPLES * 4U)
#define MPU6050_CALIBRATION_ACCEL_TOLERANCE_G 0.15
#define MPU6050_CALIBRATION_GYRO_LIMIT_DPS 3.0

/* Initial rejection limits, to be checked against real vibration logs.
 * Norm alone cannot detect every disturbance: a false gravity direction can
 * still have a magnitude of 1 g. The angle gate checks each gyro prediction.
 */
#define MPU6050_ACCEL_NORM_TOLERANCE_G 0.20
#define MPU6050_ACCEL_MAX_INNOVATION_DEG 15.0
#define MPU6050_ACCEL_RECOVERY_STABLE_S 1.0
#define MPU6050_ACCEL_RECOVERY_STABILITY_DEG 2.0
#define MPU6050_ACCEL_RECOVERY_RATE_DPS 30.0
#define MPU6050_KALMAN_BIAS_LIMIT_DPS 3.0

#if MPU6050_DLPF_HZ == 5u
#define MPU6050_DLPF_CONFIG 6U
#elif MPU6050_DLPF_HZ == 10u
#define MPU6050_DLPF_CONFIG 5U
#elif MPU6050_DLPF_HZ == 20u
#define MPU6050_DLPF_CONFIG 4U
#else /* 42 Hz; SensorConfig.h rejects unsupported choices. */
#define MPU6050_DLPF_CONFIG 3U
#endif

static const uint16_t i2c_timeout = 100U;
static uint32_t MPU6050_timer;
static uint32_t mpu_sequence;
static I2C_HandleTypeDef *mpu_i2c;
static double gyro_x_bias_dps;
static double gyro_y_bias_dps;
static double gyro_z_bias_dps;
static uint8_t mpu_ready;

static Kalman_t KalmanX = {
    .Q_angle = 0.001,
    .Q_bias = 0.003,
    .R_measure = 0.03};

static Kalman_t KalmanY = {
    .Q_angle = 0.001,
    .Q_bias = 0.003,
    .R_measure = 0.03};

static double Kalman_UpdateAngle(Kalman_t *kalman, double new_angle,
                                double new_rate, double dt,
                                uint8_t correct_angle,
                                AttitudeAxisTrace_t *trace);
static void MPU6050_ResetKalman(Kalman_t *kalman, double angle);

#if MPU6050_ACCEL_REJECTION_ENABLED
typedef struct
{
  double reference_angle;
  double stable_s;
  uint8_t recovering;
} MPU6050_AccelGate_t;

static MPU6050_AccelGate_t AccelGateX;
static MPU6050_AccelGate_t AccelGateY;

static double MPU6050_WrapAngle(double angle)
{
  while (angle > 180.0)
    angle -= 360.0;
  while (angle < -180.0)
    angle += 360.0;
  return angle;
}

static double MPU6050_FilterAngle(Kalman_t *kalman,
                                 MPU6050_AccelGate_t *gate, double accel_angle,
                                 double gyro_rate, double dt,
                                 uint8_t acceleration_valid,
                                 uint8_t recovery_allowed,
                                 AttitudeAxisTrace_t *trace, uint8_t *flags)
{
  const double predicted_angle =
      kalman->angle + dt * (gyro_rate - kalman->bias);
  const double innovation =
      MPU6050_WrapAngle(accel_angle - predicted_angle);
  if (!acceleration_valid) *flags |= ATTITUDE_TRACE_MAGNITUDE_REJECTED;
  if (fabs(innovation) > MPU6050_ACCEL_MAX_INNOVATION_DEG)
    *flags |= ATTITUDE_TRACE_INNOVATION_REJECTED;

  /* A hard angle gate alone can permanently reject the true gravity angle
   * after gyro drift. Require a full second of low gyro rates and a gravity
   * direction that stays within 2 degrees of a fixed reference before
   * recovering. Invalid magnitude, motion, or a direction change cancels it.
   */
  if (!recovery_allowed)
  {
    if (gate->recovering || fabs(innovation) > MPU6050_ACCEL_MAX_INNOVATION_DEG)
      *flags |= ATTITUDE_TRACE_RECOVERY_WAITING;
    *gate = (MPU6050_AccelGate_t){0};
  }
  else if (gate->recovering ||
           fabs(innovation) > MPU6050_ACCEL_MAX_INNOVATION_DEG)
  {
    *flags |= ATTITUDE_TRACE_RECOVERY_WAITING;
    if (gate->stable_s == 0.0 ||
        fabs(MPU6050_WrapAngle(accel_angle - gate->reference_angle)) >
            MPU6050_ACCEL_RECOVERY_STABILITY_DEG)
    {
      gate->reference_angle = accel_angle;
      gate->stable_s = dt;
      gate->recovering = 0U;
    }
    else
    {
      gate->stable_s += dt;
    }

    if (gate->stable_s >= MPU6050_ACCEL_RECOVERY_STABLE_S)
    {
      if (!gate->recovering)
      {
        /* A bias learned before lockout is no longer a trusted reference. */
        MPU6050_ResetKalman(kalman, kalman->angle);
        *flags |= ATTITUDE_TRACE_STATE_RESET;
        gate->recovering = 1U;
      }
      Kalman_UpdateAngle(kalman, accel_angle, gyro_rate, dt, 0U, trace);
      const double error = MPU6050_WrapAngle(accel_angle - kalman->angle);
      const double max_correction = MPU6050_ACCEL_RECOVERY_RATE_DPS * dt;
      const double correction = fmax(-max_correction,
                                    fmin(max_correction, error));
      const double corrected_angle = kalman->angle + correction;
      kalman->angle = MPU6050_WrapAngle(corrected_angle);
      trace->correction_deg = (float)correction;
      *flags = (uint8_t)((*flags & ~ATTITUDE_TRACE_RECOVERY_WAITING) |
                        ATTITUDE_TRACE_RECOVERY_APPLIED);
      if (kalman->angle != corrected_angle) *flags |= ATTITUDE_TRACE_ANGLE_WRAPPED;
      if (fabs(error) <= max_correction)
      {
        MPU6050_ResetKalman(kalman, kalman->angle);
        *flags |= ATTITUDE_TRACE_STATE_RESET;
        *gate = (MPU6050_AccelGate_t){0};
      }
      return kalman->angle;
    }
  }
  else
  {
    *gate = (MPU6050_AccelGate_t){0};
  }

  const uint8_t correct_angle = acceleration_valid &&
      fabs(innovation) <= MPU6050_ACCEL_MAX_INNOVATION_DEG;
  if (correct_angle) *flags |= ATTITUDE_TRACE_CORRECTION_APPLIED;

  /* Unwrap the measurement around the prediction. This handles +/-180
   * crossings without allowing the old direct angle reset to bypass gating.
   * Rejected samples still advance angle and covariance, but not gyro bias.
   */
  Kalman_UpdateAngle(kalman, predicted_angle + innovation, gyro_rate, dt,
                     correct_angle, trace);
  /* Startup calibration already removes the static gyro offset. Prevent
   * rejected/contaminated corrections from leaving a large integration bias.
   */
  kalman->bias = fmax(-MPU6050_KALMAN_BIAS_LIMIT_DPS,
                      fmin(MPU6050_KALMAN_BIAS_LIMIT_DPS, kalman->bias));
  const double unwrapped_angle = kalman->angle;
  kalman->angle = MPU6050_WrapAngle(unwrapped_angle);
  if (kalman->angle != unwrapped_angle) *flags |= ATTITUDE_TRACE_ANGLE_WRAPPED;
  return kalman->angle;
}
#endif

static HAL_StatusTypeDef I2C_ReadRegisters(uint8_t start_register,
                                           uint8_t *data, uint16_t length)
{
  if (mpu_i2c == NULL || data == NULL || length == 0U)
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, start_register,
                          I2C_MEMADD_SIZE_8BIT, data, length, i2c_timeout);
}

static void MPU6050_ResetKalman(Kalman_t *kalman, double angle)
{
  kalman->angle = angle;
  kalman->bias = 0.0;
  kalman->P[0][0] = 0.0;
  kalman->P[0][1] = 0.0;
  kalman->P[1][0] = 0.0;
  kalman->P[1][1] = 0.0;
}

static HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_t *data)
{
  if (data == NULL || mpu_i2c == NULL)
  {
    return HAL_ERROR;
  }

  uint8_t raw_data[14];
  const HAL_StatusTypeDef status =
      I2C_ReadRegisters(ACCEL_XOUT_H_REG, raw_data, sizeof(raw_data));
  if (status != HAL_OK)
  {
    return status;
  }

  data->Accel_X_RAW = (int16_t)(raw_data[0] << 8 | raw_data[1]);
  data->Accel_Y_RAW = (int16_t)(raw_data[2] << 8 | raw_data[3]);
  data->Accel_Z_RAW = (int16_t)(raw_data[4] << 8 | raw_data[5]);
  const int16_t temperature_raw =
      (int16_t)(raw_data[6] << 8 | raw_data[7]);
  data->Gyro_X_RAW = (int16_t)(raw_data[8] << 8 | raw_data[9]);
  data->Gyro_Y_RAW = (int16_t)(raw_data[10] << 8 | raw_data[11]);
  data->Gyro_Z_RAW = (int16_t)(raw_data[12] << 8 | raw_data[13]);
  data->Temperature =
      (float)((float)temperature_raw / 340.0f + 36.53f);

  return HAL_OK;
}

static void MPU6050_GetAccelerometerAngles(double acceleration_x,
                                            double acceleration_y,
                                            double acceleration_z,
                                            double *roll, double *pitch)
{
  const double roll_denominator =
      sqrt(acceleration_x * acceleration_x +
           acceleration_z * acceleration_z);
  *roll = atan2(acceleration_y, roll_denominator) * RAD_TO_DEG;
  *pitch = atan2(-acceleration_x, acceleration_z) * RAD_TO_DEG;
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *i2c)
{
  if (i2c == NULL)
  {
    return HAL_ERROR;
  }

  mpu_i2c = i2c;
  mpu_ready = 0U;
  mpu_sequence = 0U;
  gyro_x_bias_dps = 0.0;
  gyro_y_bias_dps = 0.0;
  gyro_z_bias_dps = 0.0;
  MPU6050_ResetKalman(&KalmanX, 0.0);
  MPU6050_ResetKalman(&KalmanY, 0.0);
#if MPU6050_ACCEL_REJECTION_ENABLED
  AccelGateX = (MPU6050_AccelGate_t){0};
  AccelGateY = (MPU6050_AccelGate_t){0};
#endif

  HAL_StatusTypeDef status =
      HAL_I2C_IsDeviceReady(i2c, MPU6050_ADDR, 5U, i2c_timeout);
  HAL_Delay(100U);
  if (status != HAL_OK)
  {
    return status;
  }

  uint8_t check;
  uint8_t data;
  status = HAL_I2C_Mem_Read(i2c, MPU6050_ADDR, WHO_AM_I_REG,
                            I2C_MEMADD_SIZE_8BIT, &check, 1U,
                            i2c_timeout);
  if (status != HAL_OK || check != 0x68U)
  {
    return status != HAL_OK ? status : HAL_ERROR;
  }

  data = 0x80U;
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, PWR_MGMT_1_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(100U);

  /* Wake and settle the sensor before writing its sampling/range registers.
   * This module acknowledges but ignores those writes while asleep. */
  data = 0x01U; /* X-axis gyro PLL, sleep disabled. */
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, PWR_MGMT_1_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(100U);

  // 1 kHz internal rate divided by 8 gives a 125 Hz sample rate.
  data = 7U;
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, SMPLRT_DIV_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }

  // Configurable bandwidth; every supported setting uses the 1 kHz base rate.
  data = MPU6050_DLPF_CONFIG;
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, CONFIG_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }

  // Gyroscope range: +/-250 degrees per second.
  data = 0x00U;
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, GYRO_CONFIG_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }

  // Accelerometer range: +/-8 g, with the matching conversion in SensorConfig.h.
  data = MPU6050_ACCEL_CONFIG_VALUE;
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, ACCEL_CONFIG_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }

  data = 0x04U;
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, INT_PIN_CFG_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }

  data = 0x01U;
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, INT_ENABLE_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }

  /* An acknowledged write is not proof that the range actually changed.
   * Verify the divider, filter, ranges and disabled self-test bits before
   * using their conversion scales or reporting settings in a flight log. */
  uint8_t settings[4];
  status = I2C_ReadRegisters(SMPLRT_DIV_REG, settings, sizeof(settings));
  if (status != HAL_OK)
  {
    LogError(2003, "MPU6050 configuration readback failed");
    return status;
  }
  if ((settings[3] & 0xF8U) != MPU6050_ACCEL_CONFIG_VALUE)
  {
    char message[96];
    snprintf(message, sizeof(message),
             "MPU6050 ACCEL_CONFIG mismatch: read=0x%02X expected=0x%02X",
             (unsigned)settings[3], (unsigned)MPU6050_ACCEL_CONFIG_VALUE);
    LogError(2003, message);
    return HAL_ERROR;
  }
  if (settings[0] != 7U || (settings[1] & 0x3FU) != MPU6050_DLPF_CONFIG ||
      (settings[2] & 0xF8U) != 0U)
  {
    char message[96];
    snprintf(message, sizeof(message),
             "MPU6050 config mismatch: divider=0x%02X filter=0x%02X gyro=0x%02X",
             (unsigned)settings[0], (unsigned)settings[1], (unsigned)settings[2]);
    LogError(2003, message);
    return HAL_ERROR;
  }
  LogInformation(1001, "MPU6050 sample rate, filter and +/-8 g range verified");

  MPU6050_timer = HAL_GetTick();
  LogInformation(1001, "MPU6050 started; stationary calibration required");
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Calibrate(void)
{
  if (mpu_i2c == NULL)
  {
    return HAL_ERROR;
  }

  mpu_ready = 0U;
  gyro_x_bias_dps = 0.0;
  gyro_y_bias_dps = 0.0;
  gyro_z_bias_dps = 0.0;
  MPU6050_ResetKalman(&KalmanX, 0.0);
  MPU6050_ResetKalman(&KalmanY, 0.0);
#if MPU6050_ACCEL_REJECTION_ENABLED
  AccelGateX = (MPU6050_AccelGate_t){0};
  AccelGateY = (MPU6050_AccelGate_t){0};
#endif

  LogInformation(
      1001,
      "MPU6050 calibration: keep the drone still for 3 seconds");

  MPU6050_t sample = {0};
  for (uint16_t discarded = 0U;
       discarded < MPU6050_CALIBRATION_DISCARD_SAMPLES; discarded++)
  {
    const HAL_StatusTypeDef status = MPU6050_ReadRaw(&sample);
    if (status != HAL_OK)
    {
      return status;
    }
    HAL_Delay(MPU6050_CALIBRATION_SAMPLE_DELAY_MS);
  }

  double acceleration_x_sum = 0.0;
  double acceleration_y_sum = 0.0;
  double acceleration_z_sum = 0.0;
  double gyro_x_sum = 0.0;
  double gyro_y_sum = 0.0;
  double gyro_z_sum = 0.0;
  uint16_t stationary_samples = 0U;
  uint16_t longest_stationary = 0U;
  uint16_t accel_rejected = 0U;
  uint16_t gyro_rejected = 0U;
  double min_norm = INFINITY;
  double max_norm = 0.0;
  double peak_gyro = 0.0;

  for (uint16_t attempt = 0U;
       attempt < MPU6050_CALIBRATION_MAX_ATTEMPTS &&
       stationary_samples < MPU6050_CALIBRATION_REQUIRED_SAMPLES;
       attempt++)
  {
    const HAL_StatusTypeDef status = MPU6050_ReadRaw(&sample);
    if (status != HAL_OK)
    {
      return status;
    }

    const double acceleration_x =
        (double)sample.Accel_X_RAW / MPU6050_ACCEL_SCALE_LSB_PER_G;
    const double acceleration_y =
        (double)sample.Accel_Y_RAW / MPU6050_ACCEL_SCALE_LSB_PER_G;
    const double acceleration_z =
        (double)sample.Accel_Z_RAW / MPU6050_ACCEL_SCALE_LSB_PER_G;
    const double acceleration_norm =
        sqrt(acceleration_x * acceleration_x +
             acceleration_y * acceleration_y +
             acceleration_z * acceleration_z);
    const double gyro_x =
        (double)sample.Gyro_X_RAW / MPU6050_GYRO_SCALE_LSB_PER_DPS;
    const double gyro_y =
        (double)sample.Gyro_Y_RAW / MPU6050_GYRO_SCALE_LSB_PER_DPS;
    const double gyro_z =
        (double)sample.Gyro_Z_RAW / MPU6050_GYRO_SCALE_LSB_PER_DPS;
    min_norm = fmin(min_norm, acceleration_norm);
    max_norm = fmax(max_norm, acceleration_norm);
    peak_gyro = fmax(peak_gyro, fmax(fabs(gyro_x),
                                 fmax(fabs(gyro_y), fabs(gyro_z))));
    const uint8_t accel_stationary =
        fabs(acceleration_norm - 1.0) <=
            MPU6050_CALIBRATION_ACCEL_TOLERANCE_G;
    const uint8_t gyro_stationary =
        fabs(gyro_x) <= MPU6050_CALIBRATION_GYRO_LIMIT_DPS &&
        fabs(gyro_y) <= MPU6050_CALIBRATION_GYRO_LIMIT_DPS &&
        fabs(gyro_z) <= MPU6050_CALIBRATION_GYRO_LIMIT_DPS;
    if (!accel_stationary) accel_rejected++;
    if (!gyro_stationary) gyro_rejected++;

    if (accel_stationary && gyro_stationary)
    {
      acceleration_x_sum += sample.Accel_X_RAW;
      acceleration_y_sum += sample.Accel_Y_RAW;
      acceleration_z_sum += sample.Accel_Z_RAW;
      gyro_x_sum += sample.Gyro_X_RAW;
      gyro_y_sum += sample.Gyro_Y_RAW;
      gyro_z_sum += sample.Gyro_Z_RAW;
      stationary_samples++;
      if (stationary_samples > longest_stationary)
        longest_stationary = stationary_samples;
    }
    else
    {
      // Calibration requires one uninterrupted stationary window.
      acceleration_x_sum = 0.0;
      acceleration_y_sum = 0.0;
      acceleration_z_sum = 0.0;
      gyro_x_sum = 0.0;
      gyro_y_sum = 0.0;
      gyro_z_sum = 0.0;
      stationary_samples = 0U;
    }

    HAL_Delay(MPU6050_CALIBRATION_SAMPLE_DELAY_MS);
  }

  if (stationary_samples < MPU6050_CALIBRATION_REQUIRED_SAMPLES)
  {
    char message[96];
    snprintf(message, sizeof(message),
             "MPU calibration timeout: norm=%.3f..%.3f g gyro_peak=%.2f dps",
             min_norm, max_norm, peak_gyro);
    LogError(2003, message);
    snprintf(message, sizeof(message),
             "MPU calibration rejected: accel=%u gyro=%u longest=%u/%u samples",
             (unsigned)accel_rejected, (unsigned)gyro_rejected,
             (unsigned)longest_stationary,
             (unsigned)MPU6050_CALIBRATION_REQUIRED_SAMPLES);
    LogError(2003, message);
    return HAL_TIMEOUT;
  }

  const double sample_count = (double)stationary_samples;
  const double acceleration_x_average = acceleration_x_sum / sample_count;
  const double acceleration_y_average = acceleration_y_sum / sample_count;
  const double acceleration_z_average = acceleration_z_sum / sample_count;
  double initial_roll;
  double initial_pitch;
  MPU6050_GetAccelerometerAngles(acceleration_x_average,
                                  acceleration_y_average,
                                  acceleration_z_average,
                                  &initial_roll, &initial_pitch);

  gyro_x_bias_dps =
      (gyro_x_sum / sample_count) / MPU6050_GYRO_SCALE_LSB_PER_DPS;
  gyro_y_bias_dps =
      (gyro_y_sum / sample_count) / MPU6050_GYRO_SCALE_LSB_PER_DPS;
  gyro_z_bias_dps =
      (gyro_z_sum / sample_count) / MPU6050_GYRO_SCALE_LSB_PER_DPS;
  MPU6050_ResetKalman(&KalmanX, initial_roll);
  MPU6050_ResetKalman(&KalmanY, initial_pitch);
  MPU6050_timer = HAL_GetTick();
  mpu_ready = 1U;
  LogInformation(1001, "MPU6050 stationary calibration completed");

  return HAL_OK;
}

uint8_t MPU6050_IsReady(void)
{
  return mpu_ready;
}

HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_t *data)
{
  if (data == NULL || mpu_i2c == NULL)
  {
    return HAL_ERROR;
  }
  if (!mpu_ready)
  {
    return HAL_BUSY;
  }

  const HAL_StatusTypeDef status = MPU6050_ReadRaw(data);
  if (status != HAL_OK)
  {
    return status;
  }

  data->Ax = data->Accel_X_RAW / MPU6050_ACCEL_SCALE_LSB_PER_G;
  data->Ay = data->Accel_Y_RAW / MPU6050_ACCEL_SCALE_LSB_PER_G;
  data->Az = data->Accel_Z_RAW / MPU6050_ACCEL_SCALE_LSB_PER_G;

  // Convert from g to m/s^2 for all axes.
  data->Ax_ms = data->Ax * 9.80665;
  data->Ay_ms = data->Ay * 9.80665;
  data->Az_ms = data->Az * 9.80665;

  data->Gx = data->Gyro_X_RAW / MPU6050_GYRO_SCALE_LSB_PER_DPS -
             gyro_x_bias_dps;
  data->Gy = data->Gyro_Y_RAW / MPU6050_GYRO_SCALE_LSB_PER_DPS -
             gyro_y_bias_dps;
  data->Gz = data->Gyro_Z_RAW / MPU6050_GYRO_SCALE_LSB_PER_DPS -
             gyro_z_bias_dps;

  const uint32_t now = HAL_GetTick();
  double dt = (double)(now - MPU6050_timer) / 1000.0;
  MPU6050_timer = now;
  uint8_t timing_flags = 0U;
  if (dt <= 0.0)
  {
    dt = 0.001;
    timing_flags |= ATTITUDE_TRACE_DT_ZERO_CLAMPED;
  }
  if (dt > 0.1)
  {
    dt = 0.1;
    timing_flags |= ATTITUDE_TRACE_DT_HIGH_CLAMPED;
  }
  if (data->Gyro_X_RAW == INT16_MIN || data->Gyro_X_RAW == INT16_MAX ||
      data->Gyro_Y_RAW == INT16_MIN || data->Gyro_Y_RAW == INT16_MAX ||
      data->Gyro_Z_RAW == INT16_MIN || data->Gyro_Z_RAW == INT16_MAX)
    timing_flags |= ATTITUDE_TRACE_GYRO_CLIPPED;
  if (data->Accel_X_RAW == INT16_MIN || data->Accel_X_RAW == INT16_MAX ||
      data->Accel_Y_RAW == INT16_MIN || data->Accel_Y_RAW == INT16_MAX ||
      data->Accel_Z_RAW == INT16_MIN || data->Accel_Z_RAW == INT16_MAX)
    timing_flags |= ATTITUDE_TRACE_ACCEL_CLIPPED;
  data->attitude_trace = (AttitudeTrace_t){
      .sequence = ++mpu_sequence, .timestamp_ms = now,
      .dt_ms = (float)(dt * 1000.0), .accel_x_g = (float)data->Ax,
      .accel_y_g = (float)data->Ay, .accel_z_g = (float)data->Az,
      .valid = 1U, .timing_flags = timing_flags};
  AttitudeTrace_t *trace = &data->attitude_trace;

  double roll;
  double pitch;
  MPU6050_GetAccelerometerAngles(data->Accel_X_RAW, data->Accel_Y_RAW,
                                  data->Accel_Z_RAW, &roll, &pitch);
  trace->roll.accel_deg = (float)pitch;
  trace->pitch.accel_deg = (float)roll;
#if MPU6050_ACCEL_REJECTION_ENABLED
  const double acceleration_norm =
      sqrt(data->Ax * data->Ax + data->Ay * data->Ay + data->Az * data->Az);
  const uint8_t acceleration_valid =
      fabs(acceleration_norm - 1.0) <= MPU6050_ACCEL_NORM_TOLERANCE_G;
  const uint8_t recovery_allowed =
      fabs(acceleration_norm - 1.0) <= MPU6050_CALIBRATION_ACCEL_TOLERANCE_G &&
      fabs(data->Gx) <= MPU6050_CALIBRATION_GYRO_LIMIT_DPS &&
      fabs(data->Gy) <= MPU6050_CALIBRATION_GYRO_LIMIT_DPS &&
      fabs(data->Gz) <= MPU6050_CALIBRATION_GYRO_LIMIT_DPS;
  data->KalmanAngleY =
      MPU6050_FilterAngle(&KalmanY, &AccelGateY, pitch, data->Gy, dt,
                           acceleration_valid, recovery_allowed,
                           &trace->roll, &trace->roll_flags);
#else
  if ((pitch < -90.0 && KalmanY.angle > 90.0) ||
      (pitch > 90.0 && KalmanY.angle < -90.0))
  {
    trace->roll.previous_deg = (float)KalmanY.angle;
    trace->roll.bias_dps = (float)KalmanY.bias;
    const double prediction = KalmanY.angle + dt * (data->Gy - KalmanY.bias);
    trace->roll.predicted_deg = (float)prediction;
    trace->roll.correction_deg = (float)(pitch - prediction);
    trace->roll_flags = ATTITUDE_TRACE_LEGACY_RESET;
    KalmanY.angle = pitch;
    data->KalmanAngleY = pitch;
  }
  else
  {
    data->KalmanAngleY =
        Kalman_UpdateAngle(&KalmanY, pitch, data->Gy, dt, 1U, &trace->roll);
    trace->roll_flags = ATTITUDE_TRACE_CORRECTION_APPLIED;
  }
#endif

  if (fabs(data->KalmanAngleY) > 90.0)
  {
    data->Gx = -data->Gx;
  }
#if MPU6050_ACCEL_REJECTION_ENABLED
  data->KalmanAngleX =
      MPU6050_FilterAngle(&KalmanX, &AccelGateX, roll, data->Gx, dt,
                           acceleration_valid, recovery_allowed,
                           &trace->pitch, &trace->pitch_flags);
#else
  data->KalmanAngleX = Kalman_UpdateAngle(&KalmanX, roll, data->Gx, dt,
                                          1U, &trace->pitch);
  trace->pitch_flags = ATTITUDE_TRACE_CORRECTION_APPLIED;
#endif

  return HAL_OK;
}

double Kalman_GetAngle(Kalman_t *kalman, double new_angle, double new_rate,
                       double dt)
{
  return Kalman_UpdateAngle(kalman, new_angle, new_rate, dt, 1U, NULL);
}

static double Kalman_UpdateAngle(Kalman_t *kalman, double new_angle,
                                double new_rate, double dt,
                                uint8_t correct_angle,
                                AttitudeAxisTrace_t *trace)
{
  const double rate = new_rate - kalman->bias;
  if (trace != NULL)
  {
    trace->previous_deg = (float)kalman->angle;
    trace->bias_dps = (float)kalman->bias;
    trace->correction_deg = 0.0f;
  }
  kalman->angle += dt * rate;
  if (trace != NULL) trace->predicted_deg = (float)kalman->angle;

  kalman->P[0][0] +=
      dt * (dt * kalman->P[1][1] - kalman->P[0][1] -
            kalman->P[1][0] + kalman->Q_angle);
  kalman->P[0][1] -= dt * kalman->P[1][1];
  kalman->P[1][0] -= dt * kalman->P[1][1];
  kalman->P[1][1] += kalman->Q_bias * dt;

  if (!correct_angle)
  {
    return kalman->angle;
  }

  const double innovation_covariance =
      kalman->P[0][0] + kalman->R_measure;
  const double gain_angle = kalman->P[0][0] / innovation_covariance;
  const double gain_bias = kalman->P[1][0] / innovation_covariance;

  const double innovation = new_angle - kalman->angle;
  if (trace != NULL) trace->correction_deg = (float)(gain_angle * innovation);
  kalman->angle += gain_angle * innovation;
  kalman->bias += gain_bias * innovation;

  const double covariance_angle = kalman->P[0][0];
  const double covariance_angle_bias = kalman->P[0][1];
  kalman->P[0][0] -= gain_angle * covariance_angle;
  kalman->P[0][1] -= gain_angle * covariance_angle_bias;
  kalman->P[1][0] -= gain_bias * covariance_angle;
  kalman->P[1][1] -= gain_bias * covariance_angle_bias;

  return kalman->angle;
}
