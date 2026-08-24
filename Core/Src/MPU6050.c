// Read data from consecutive registers of an I2C device
#include "MPU6050.h"
#include "Tools.h"
#include <math.h>

#define MPU6050_ACCEL_SCALE_LSB_PER_G 16384.0
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

static const uint16_t i2c_timeout = 100U;
static uint32_t MPU6050_timer;
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
  gyro_x_bias_dps = 0.0;
  gyro_y_bias_dps = 0.0;
  gyro_z_bias_dps = 0.0;
  MPU6050_ResetKalman(&KalmanX, 0.0);
  MPU6050_ResetKalman(&KalmanY, 0.0);

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

  // 1 kHz internal rate divided by 8 gives a 125 Hz sample rate.
  data = 7U;
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, SMPLRT_DIV_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }

  // Enable the digital low-pass filter.
  data = 5U;
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

  // Accelerometer range: +/-2 g.
  data = 0x00U;
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

  // Wake the sensor and use the X-axis gyro PLL as the clock source.
  data = 0x01U;
  status = HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, PWR_MGMT_1_REG,
                             I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
  if (status != HAL_OK)
  {
    return status;
  }
  HAL_Delay(100U);

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
    const uint8_t stationary =
        fabs(acceleration_norm - 1.0) <=
            MPU6050_CALIBRATION_ACCEL_TOLERANCE_G &&
        fabs(gyro_x) <= MPU6050_CALIBRATION_GYRO_LIMIT_DPS &&
        fabs(gyro_y) <= MPU6050_CALIBRATION_GYRO_LIMIT_DPS &&
        fabs(gyro_z) <= MPU6050_CALIBRATION_GYRO_LIMIT_DPS;

    if (stationary)
    {
      acceleration_x_sum += sample.Accel_X_RAW;
      acceleration_y_sum += sample.Accel_Y_RAW;
      acceleration_z_sum += sample.Accel_Z_RAW;
      gyro_x_sum += sample.Gyro_X_RAW;
      gyro_y_sum += sample.Gyro_Y_RAW;
      gyro_z_sum += sample.Gyro_Z_RAW;
      stationary_samples++;
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
  if (dt <= 0.0)
  {
    dt = 0.001;
  }
  if (dt > 0.1)
  {
    dt = 0.1;
  }

  double roll;
  double pitch;
  MPU6050_GetAccelerometerAngles(data->Accel_X_RAW, data->Accel_Y_RAW,
                                  data->Accel_Z_RAW, &roll, &pitch);
  if ((pitch < -90.0 && KalmanY.angle > 90.0) ||
      (pitch > 90.0 && KalmanY.angle < -90.0))
  {
    KalmanY.angle = pitch;
    data->KalmanAngleY = pitch;
  }
  else
  {
    data->KalmanAngleY =
        Kalman_GetAngle(&KalmanY, pitch, data->Gy, dt);
  }

  if (fabs(data->KalmanAngleY) > 90.0)
  {
    data->Gx = -data->Gx;
  }
  data->KalmanAngleX =
      Kalman_GetAngle(&KalmanX, roll, data->Gx, dt);

  return HAL_OK;
}

double Kalman_GetAngle(Kalman_t *kalman, double new_angle, double new_rate,
                       double dt)
{
  const double rate = new_rate - kalman->bias;
  kalman->angle += dt * rate;

  kalman->P[0][0] +=
      dt * (dt * kalman->P[1][1] - kalman->P[0][1] -
            kalman->P[1][0] + kalman->Q_angle);
  kalman->P[0][1] -= dt * kalman->P[1][1];
  kalman->P[1][0] -= dt * kalman->P[1][1];
  kalman->P[1][1] += kalman->Q_bias * dt;

  const double innovation_covariance =
      kalman->P[0][0] + kalman->R_measure;
  const double gain_angle = kalman->P[0][0] / innovation_covariance;
  const double gain_bias = kalman->P[1][0] / innovation_covariance;

  const double innovation = new_angle - kalman->angle;
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
