#include "VelocityEstimator.h"

#include "SensorConfig.h"
#include <math.h>
#include <stddef.h>

#define VELOCITY_ESTIMATOR_GZ_BIAS_STATIONARY_THRESHOLD_DPS 1.5
#define VELOCITY_ESTIMATOR_GZ_BIAS_ALPHA 0.01
#define VELOCITY_ESTIMATOR_GZ_BIAS_MIN_STATIONARY_SAMPLES 5U
#define VELOCITY_ESTIMATOR_SRF05_SAMPLE_MS 50U
#define VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS 300U
#define VELOCITY_ESTIMATOR_SRF05_MIN_TILT_SCALE 0.80
#define VELOCITY_ESTIMATOR_SRF05_MEDIAN_SAMPLES 3U
#define VELOCITY_ESTIMATOR_SRF05_MEDIAN_GATE_M 0.08
#define VELOCITY_ESTIMATOR_SRF05_MOTION_GATE_ACCEL_MS2 4.0
#define VELOCITY_ESTIMATOR_SRF05_ACQUIRE_SAMPLES 8U
#define VELOCITY_ESTIMATOR_SRF05_REACQUIRE_OUTLIERS 4U
#define VELOCITY_ESTIMATOR_SRF05_MEASUREMENT_NOISE_M 0.01
#define VELOCITY_ESTIMATOR_SRF05_MIN_INNOVATION_GATE_M 0.08
#define VELOCITY_ESTIMATOR_SRF05_INNOVATION_SIGMA 5.0
#define VELOCITY_ESTIMATOR_ACCEL_PROCESS_NOISE_MS2 0.8
#define VELOCITY_ESTIMATOR_BIAS_RANDOM_WALK_MS2 0.02
#define VELOCITY_ESTIMATOR_MAX_RESIDUAL_BIAS_MS2 1.0
#define VELOCITY_ESTIMATOR_BMP280_SAMPLE_MS 100U
#define VELOCITY_ESTIMATOR_GRAVITY_MS2 9.80665
#define VELOCITY_ESTIMATOR_ACCEL_NORM_TOLERANCE_MS2 1.5
#define VELOCITY_ESTIMATOR_GYRO_STATIONARY_THRESHOLD_DPS 3.0
#define VELOCITY_ESTIMATOR_ACCEL_DEADBAND_MS2 0.12
#define VELOCITY_ESTIMATOR_MAX_ACCEL_MS2 30.0
#define VELOCITY_ESTIMATOR_MAX_SPEED_MS 15.0
#define VELOCITY_ESTIMATOR_MAX_ALTITUDE_SPEED_MS 8.0
#define VELOCITY_ESTIMATOR_ALTITUDE_FILTER_TAU_S 0.4
#define VELOCITY_ESTIMATOR_FUSION_TAU_S 1.0
#define VELOCITY_ESTIMATOR_ALTITUDE_MAX_GAP_MS 1000U
#define VELOCITY_ESTIMATOR_PI 3.14159265358979323846

static VelocityEstimatorEstimate_t estimate;

static double gyro_z_bias_dps;
static uint16_t gyro_z_stationary_samples;
static uint8_t gyro_z_bias_initialized;
static double vertical_acceleration_ms2;
static double vertical_acceleration_bias_ms2;
static double vertical_tilt_scale;
static uint8_t vertical_acceleration_bias_samples;
static uint8_t vertical_acceleration_bias_initialized;

static double previous_vertical_altitude_m;
static double filtered_altitude_velocity_ms;
static uint32_t previous_vertical_altitude_ms;
static uint32_t last_bmp280_sample_ms;
static uint8_t have_previous_vertical_altitude;

enum
{
  VELOCITY_ESTIMATOR_STATE_ALTITUDE = 0,
  VELOCITY_ESTIMATOR_STATE_VELOCITY,
  VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS,
  VELOCITY_ESTIMATOR_STATE_COUNT
};

static double velocity_state[VELOCITY_ESTIMATOR_STATE_COUNT];
static double velocity_covariance[VELOCITY_ESTIMATOR_STATE_COUNT]
                                  [VELOCITY_ESTIMATOR_STATE_COUNT];
static double srf05_median_altitude_m[
    VELOCITY_ESTIMATOR_SRF05_MEDIAN_SAMPLES];
static uint8_t srf05_median_count;
static uint8_t srf05_median_index;
static uint8_t srf05_filter_initialized;
static uint8_t srf05_filter_inliers;
static uint8_t srf05_filter_outliers;
static uint32_t last_srf05_filter_update_ms;
static uint32_t last_srf05_raw_sample_ms;

static double VelocityEstimator_Clamp(double value, double minimum,
                                      double maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static void VelocityEstimator_ResetGyroZ(void)
{
  gyro_z_bias_dps = 0.0;
  gyro_z_stationary_samples = 0U;
  gyro_z_bias_initialized = 0U;
}

static void VelocityEstimator_ResetSrf05Filter(void)
{
  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    velocity_state[row] = 0.0;
    for (uint8_t column = 0U; column < VELOCITY_ESTIMATOR_STATE_COUNT;
         column++)
    {
      velocity_covariance[row][column] = 0.0;
    }
  }

  for (uint8_t sample = 0U;
       sample < VELOCITY_ESTIMATOR_SRF05_MEDIAN_SAMPLES; sample++)
  {
    srf05_median_altitude_m[sample] = 0.0;
  }

  srf05_median_count = 0U;
  srf05_median_index = 0U;
  srf05_filter_initialized = 0U;
  srf05_filter_inliers = 0U;
  srf05_filter_outliers = 0U;
  last_srf05_filter_update_ms = 0U;
  last_srf05_raw_sample_ms = 0U;
  estimate.vertical_velocity_ms = 0.0;
  estimate.vertical_velocity_valid = 0U;
}

void VelocityEstimator_ResetVertical(void)
{
  estimate.vertical_velocity_ms = 0.0;
  estimate.vertical_velocity_valid = 0U;
  estimate.srf05_altitude_valid = 0U;

  if (SENSOR_MPU6050_ENABLED)
  {
    vertical_acceleration_ms2 = 0.0;
    vertical_acceleration_bias_ms2 = 0.0;
    vertical_tilt_scale = 1.0;
    vertical_acceleration_bias_samples = 0U;
    vertical_acceleration_bias_initialized = 0U;
  }

  if (SENSOR_BMP280_ENABLED && !SENSOR_SRF05_ENABLED)
  {
    previous_vertical_altitude_m = 0.0;
    filtered_altitude_velocity_ms = 0.0;
    previous_vertical_altitude_ms = 0U;
    last_bmp280_sample_ms = 0U;
    have_previous_vertical_altitude = 0U;
  }

  if (SENSOR_SRF05_ENABLED)
  {
    VelocityEstimator_ResetSrf05Filter();
  }
}

void VelocityEstimator_Init(void)
{
  if (SENSOR_MPU6050_ENABLED)
  {
    VelocityEstimator_ResetGyroZ();
  }
  estimate.srf05_altitude_cm = 0.0;
  VelocityEstimator_ResetVertical();
}

double VelocityEstimator_CorrectGyroZ(double raw_gyro_z_dps)
{
  if (SENSOR_MPU6050_ENABLED)
  {
    if (fabs(raw_gyro_z_dps) <
        VELOCITY_ESTIMATOR_GZ_BIAS_STATIONARY_THRESHOLD_DPS)
    {
      if (gyro_z_stationary_samples <
          VELOCITY_ESTIMATOR_GZ_BIAS_MIN_STATIONARY_SAMPLES)
      {
        gyro_z_stationary_samples++;
      }

      if (!gyro_z_bias_initialized)
      {
        gyro_z_bias_dps +=
            (raw_gyro_z_dps - gyro_z_bias_dps) /
            (double)gyro_z_stationary_samples;
        if (gyro_z_stationary_samples >=
            VELOCITY_ESTIMATOR_GZ_BIAS_MIN_STATIONARY_SAMPLES)
        {
          gyro_z_bias_initialized = 1U;
        }
      }
      else
      {
        gyro_z_bias_dps += VELOCITY_ESTIMATOR_GZ_BIAS_ALPHA *
                           (raw_gyro_z_dps - gyro_z_bias_dps);
      }
    }
    else
    {
      if (!gyro_z_bias_initialized)
      {
        gyro_z_bias_dps = 0.0;
      }
      gyro_z_stationary_samples = 0U;
    }

    if (gyro_z_bias_initialized)
    {
      return raw_gyro_z_dps - gyro_z_bias_dps;
    }
  }

  return raw_gyro_z_dps;
}

void VelocityEstimator_UpdateImu(
    const VelocityEstimatorImuSample_t *imu_sample)
{
  if (!SENSOR_MPU6050_ENABLED)
  {
    (void)imu_sample;
    return;
  }

  if (imu_sample == NULL)
  {
    vertical_acceleration_ms2 = 0.0;
    return;
  }

  const double mpu_angle_x_rad =
      imu_sample->mpu_angle_x_deg * VELOCITY_ESTIMATOR_PI / 180.0;
  const double mpu_angle_y_rad =
      imu_sample->mpu_angle_y_deg * VELOCITY_ESTIMATOR_PI / 180.0;
  const double sin_angle_x = sin(mpu_angle_x_rad);
  const double cos_angle_x = cos(mpu_angle_x_rad);
  const double sin_angle_y = sin(mpu_angle_y_rad);
  const double cos_angle_y = cos(mpu_angle_y_rad);

  vertical_tilt_scale = VelocityEstimator_Clamp(cos_angle_x * cos_angle_y,
                                                 0.0, 1.0);

  /* MPU6050_ReadAll defines angle X as
   * atan(Ay / sqrt(Ax^2 + Az^2)) and angle Y as atan2(-Ax, Az).
   * These definitions imply the sensor-frame upward unit vector
   * [-cos(X)sin(Y), sin(X), cos(X)cos(Y)]. */
  const double specific_force_up =
      (-cos_angle_x * sin_angle_y *
       imu_sample->acceleration_x_ms2) +
      (sin_angle_x * imu_sample->acceleration_y_ms2) +
      (cos_angle_x * cos_angle_y *
       imu_sample->acceleration_z_ms2);
  const double raw_vertical_acceleration_ms2 =
      specific_force_up - VELOCITY_ESTIMATOR_GRAVITY_MS2;

  if (!isfinite(raw_vertical_acceleration_ms2))
  {
    vertical_acceleration_ms2 = 0.0;
    return;
  }

  if (!vertical_acceleration_bias_initialized)
  {
    const double acceleration_norm_ms2 =
        sqrt((imu_sample->acceleration_x_ms2 *
              imu_sample->acceleration_x_ms2) +
             (imu_sample->acceleration_y_ms2 *
              imu_sample->acceleration_y_ms2) +
             (imu_sample->acceleration_z_ms2 *
              imu_sample->acceleration_z_ms2));
    const uint8_t stationary =
        fabs(acceleration_norm_ms2 - VELOCITY_ESTIMATOR_GRAVITY_MS2) <=
            VELOCITY_ESTIMATOR_ACCEL_NORM_TOLERANCE_MS2 &&
        fabs(imu_sample->gyro_x_dps) <=
            VELOCITY_ESTIMATOR_GYRO_STATIONARY_THRESHOLD_DPS &&
        fabs(imu_sample->gyro_y_dps) <=
            VELOCITY_ESTIMATOR_GYRO_STATIONARY_THRESHOLD_DPS &&
        fabs(imu_sample->gyro_z_dps) <=
            VELOCITY_ESTIMATOR_GYRO_STATIONARY_THRESHOLD_DPS;

    if (!stationary)
    {
      vertical_acceleration_bias_ms2 = 0.0;
      vertical_acceleration_bias_samples = 0U;
      vertical_acceleration_ms2 = 0.0;
      return;
    }

    vertical_acceleration_bias_samples++;
    vertical_acceleration_bias_ms2 +=
        (raw_vertical_acceleration_ms2 -
         vertical_acceleration_bias_ms2) /
        (double)vertical_acceleration_bias_samples;

    if (vertical_acceleration_bias_samples <
        VELOCITY_ESTIMATOR_ACCEL_BIAS_SAMPLE_COUNT)
    {
      vertical_acceleration_ms2 = 0.0;
      return;
    }

    vertical_acceleration_bias_initialized = 1U;
  }

  vertical_acceleration_ms2 =
      raw_vertical_acceleration_ms2 - vertical_acceleration_bias_ms2;
}

static uint8_t VelocityEstimator_FilterSrf05Altitude(
    double altitude_m, uint32_t sample_time_ms, double *filtered_altitude_m)
{
  if (filtered_altitude_m == NULL || !isfinite(altitude_m))
  {
    return 0U;
  }

  uint32_t elapsed_ms = VELOCITY_ESTIMATOR_SRF05_SAMPLE_MS;
  if (last_srf05_raw_sample_ms != 0U)
  {
    elapsed_ms = sample_time_ms - last_srf05_raw_sample_ms;
  }
  last_srf05_raw_sample_ms = sample_time_ms;

  srf05_median_altitude_m[srf05_median_index] = altitude_m;
  srf05_median_index =
      (uint8_t)((srf05_median_index + 1U) %
                VELOCITY_ESTIMATOR_SRF05_MEDIAN_SAMPLES);
  if (srf05_median_count < VELOCITY_ESTIMATOR_SRF05_MEDIAN_SAMPLES)
  {
    srf05_median_count++;
  }

  if (srf05_median_count < VELOCITY_ESTIMATOR_SRF05_MEDIAN_SAMPLES)
  {
    return 0U;
  }

  const double first = srf05_median_altitude_m[0];
  const double second = srf05_median_altitude_m[1];
  const double third = srf05_median_altitude_m[2];

  if ((first >= second && first <= third) ||
      (first <= second && first >= third))
  {
    *filtered_altitude_m = first;
  }
  else if ((second >= first && second <= third) ||
           (second <= first && second >= third))
  {
    *filtered_altitude_m = second;
  }
  else
  {
    *filtered_altitude_m = third;
  }

  const double elapsed_s =
      (double)((elapsed_ms > VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS)
                   ? VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS
                   : elapsed_ms) /
      1000.0;
  double motion_gate_m = VELOCITY_ESTIMATOR_SRF05_MEDIAN_GATE_M;
  if (srf05_filter_initialized)
  {
    motion_gate_m +=
        fabs(velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY]) *
            elapsed_s +
        0.5 * VELOCITY_ESTIMATOR_SRF05_MOTION_GATE_ACCEL_MS2 *
            elapsed_s * elapsed_s;
  }

  if (fabs(altitude_m - *filtered_altitude_m) > motion_gate_m)
  {
    return 0U;
  }

  *filtered_altitude_m = altitude_m;
  return 1U;
}

static void VelocityEstimator_InitializeSrf05Filter(
    double altitude_m, uint32_t sample_time_ms)
{
  velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE] = altitude_m;
  velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY] = 0.0;
  velocity_state[VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS] = 0.0;

  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    for (uint8_t column = 0U; column < VELOCITY_ESTIMATOR_STATE_COUNT;
         column++)
    {
      velocity_covariance[row][column] = 0.0;
    }
  }

  const double altitude_variance =
      VELOCITY_ESTIMATOR_SRF05_MEASUREMENT_NOISE_M *
      VELOCITY_ESTIMATOR_SRF05_MEASUREMENT_NOISE_M;
  velocity_covariance[VELOCITY_ESTIMATOR_STATE_ALTITUDE]
                     [VELOCITY_ESTIMATOR_STATE_ALTITUDE] =
      altitude_variance;
  velocity_covariance[VELOCITY_ESTIMATOR_STATE_VELOCITY]
                     [VELOCITY_ESTIMATOR_STATE_VELOCITY] = 0.25;
  if (SENSOR_MPU6050_ENABLED)
  {
    velocity_covariance[VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS]
                       [VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS] = 0.04;
  }

  srf05_filter_initialized = 1U;
  srf05_filter_inliers = 1U;
  srf05_filter_outliers = 0U;
  last_srf05_filter_update_ms = sample_time_ms;
  estimate.vertical_velocity_ms = 0.0;
  estimate.vertical_velocity_valid = 0U;
}

static void VelocityEstimator_PredictSrf05(double dt, uint32_t now_ms)
{
  if (!srf05_filter_initialized)
  {
    estimate.vertical_velocity_ms = 0.0;
    estimate.vertical_velocity_valid = 0U;
    return;
  }

  if ((now_ms - last_srf05_filter_update_ms) >
      VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS)
  {
    VelocityEstimator_ResetSrf05Filter();
    return;
  }

  double acceleration_ms2 = 0.0;
  if (SENSOR_MPU6050_ENABLED)
  {
    acceleration_ms2 = VelocityEstimator_Clamp(
        vertical_acceleration_ms2, -VELOCITY_ESTIMATOR_MAX_ACCEL_MS2,
        VELOCITY_ESTIMATOR_MAX_ACCEL_MS2);
  }

  const double half_dt_squared = 0.5 * dt * dt;
  const double corrected_acceleration_ms2 =
      acceleration_ms2 -
      velocity_state[VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS];

  velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE] +=
      velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY] * dt +
      corrected_acceleration_ms2 * half_dt_squared;
  velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY] +=
      corrected_acceleration_ms2 * dt;

  double transition[VELOCITY_ESTIMATOR_STATE_COUNT]
                   [VELOCITY_ESTIMATOR_STATE_COUNT] = {
                       {1.0, dt, 0.0},
                       {0.0, 1.0, 0.0},
                       {0.0, 0.0, 1.0}};
  if (SENSOR_MPU6050_ENABLED)
  {
    transition[VELOCITY_ESTIMATOR_STATE_ALTITUDE]
              [VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS] = -half_dt_squared;
    transition[VELOCITY_ESTIMATOR_STATE_VELOCITY]
              [VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS] = -dt;
  }

  double transition_covariance[VELOCITY_ESTIMATOR_STATE_COUNT]
                              [VELOCITY_ESTIMATOR_STATE_COUNT] = {{0.0}};
  double predicted_covariance[VELOCITY_ESTIMATOR_STATE_COUNT]
                             [VELOCITY_ESTIMATOR_STATE_COUNT] = {{0.0}};

  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    for (uint8_t column = 0U; column < VELOCITY_ESTIMATOR_STATE_COUNT;
         column++)
    {
      for (uint8_t index = 0U; index < VELOCITY_ESTIMATOR_STATE_COUNT;
           index++)
      {
        transition_covariance[row][column] +=
            transition[row][index] * velocity_covariance[index][column];
      }
    }
  }

  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    for (uint8_t column = 0U; column < VELOCITY_ESTIMATOR_STATE_COUNT;
         column++)
    {
      for (uint8_t index = 0U; index < VELOCITY_ESTIMATOR_STATE_COUNT;
           index++)
      {
        predicted_covariance[row][column] +=
            transition_covariance[row][index] * transition[column][index];
      }
    }
  }

  const double acceleration_variance =
      VELOCITY_ESTIMATOR_ACCEL_PROCESS_NOISE_MS2 *
      VELOCITY_ESTIMATOR_ACCEL_PROCESS_NOISE_MS2;
  predicted_covariance[VELOCITY_ESTIMATOR_STATE_ALTITUDE]
                      [VELOCITY_ESTIMATOR_STATE_ALTITUDE] +=
      half_dt_squared * half_dt_squared * acceleration_variance;
  predicted_covariance[VELOCITY_ESTIMATOR_STATE_ALTITUDE]
                      [VELOCITY_ESTIMATOR_STATE_VELOCITY] +=
      half_dt_squared * dt * acceleration_variance;
  predicted_covariance[VELOCITY_ESTIMATOR_STATE_VELOCITY]
                      [VELOCITY_ESTIMATOR_STATE_ALTITUDE] +=
      half_dt_squared * dt * acceleration_variance;
  predicted_covariance[VELOCITY_ESTIMATOR_STATE_VELOCITY]
                      [VELOCITY_ESTIMATOR_STATE_VELOCITY] +=
      dt * dt * acceleration_variance;
  if (SENSOR_MPU6050_ENABLED)
  {
    const double bias_random_walk_variance =
        VELOCITY_ESTIMATOR_BIAS_RANDOM_WALK_MS2 *
        VELOCITY_ESTIMATOR_BIAS_RANDOM_WALK_MS2 * dt;
    predicted_covariance[VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS]
                        [VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS] +=
        bias_random_walk_variance;
  }

  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    for (uint8_t column = 0U; column < VELOCITY_ESTIMATOR_STATE_COUNT;
         column++)
    {
      velocity_covariance[row][column] =
          predicted_covariance[row][column];
    }
  }

  velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY] =
      VelocityEstimator_Clamp(
          velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY],
          -VELOCITY_ESTIMATOR_MAX_SPEED_MS,
          VELOCITY_ESTIMATOR_MAX_SPEED_MS);
  velocity_state[VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS] =
      VelocityEstimator_Clamp(
          velocity_state[VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS],
          -VELOCITY_ESTIMATOR_MAX_RESIDUAL_BIAS_MS2,
          VELOCITY_ESTIMATOR_MAX_RESIDUAL_BIAS_MS2);
  estimate.vertical_velocity_ms =
      velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY];
  if (estimate.srf05_altitude_valid)
  {
    estimate.srf05_altitude_cm =
        velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE] * 100.0;
  }
}

static uint8_t VelocityEstimator_UpdateSrf05Filter(
    double altitude_m, uint32_t sample_time_ms)
{
  double median_altitude_m;
  if (!VelocityEstimator_FilterSrf05Altitude(
          altitude_m, sample_time_ms, &median_altitude_m))
  {
    return 0U;
  }

  if (srf05_filter_initialized &&
      (sample_time_ms - last_srf05_filter_update_ms) >
          VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS)
  {
    VelocityEstimator_ResetSrf05Filter();
  }

  if (!srf05_filter_initialized)
  {
    VelocityEstimator_InitializeSrf05Filter(median_altitude_m,
                                            sample_time_ms);
    return 1U;
  }

  const double measurement_variance =
      VELOCITY_ESTIMATOR_SRF05_MEASUREMENT_NOISE_M *
      VELOCITY_ESTIMATOR_SRF05_MEASUREMENT_NOISE_M;
  const double innovation =
      median_altitude_m -
      velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE];
  const double innovation_variance =
      velocity_covariance[VELOCITY_ESTIMATOR_STATE_ALTITUDE]
                         [VELOCITY_ESTIMATOR_STATE_ALTITUDE] +
      measurement_variance;
  if (!isfinite(innovation) || !isfinite(innovation_variance) ||
      innovation_variance <= 0.0)
  {
    return 0U;
  }

  const double innovation_gate = fmax(
      VELOCITY_ESTIMATOR_SRF05_MIN_INNOVATION_GATE_M,
      VELOCITY_ESTIMATOR_SRF05_INNOVATION_SIGMA *
          sqrt(innovation_variance));
  if (fabs(innovation) > innovation_gate)
  {
    if (srf05_filter_outliers < UINT8_MAX)
    {
      srf05_filter_outliers++;
    }
    if (srf05_filter_outliers >=
        VELOCITY_ESTIMATOR_SRF05_REACQUIRE_OUTLIERS)
    {
      VelocityEstimator_ResetSrf05Filter();
      VelocityEstimator_InitializeSrf05Filter(median_altitude_m,
                                              sample_time_ms);
      return 1U;
    }
    return 0U;
  }

  double kalman_gain[VELOCITY_ESTIMATOR_STATE_COUNT];
  for (uint8_t index = 0U; index < VELOCITY_ESTIMATOR_STATE_COUNT; index++)
  {
    kalman_gain[index] =
        velocity_covariance[index]
                           [VELOCITY_ESTIMATOR_STATE_ALTITUDE] /
        innovation_variance;
  }

  for (uint8_t index = 0U; index < VELOCITY_ESTIMATOR_STATE_COUNT; index++)
  {
    velocity_state[index] += kalman_gain[index] * innovation;
  }

  double residual_transform[VELOCITY_ESTIMATOR_STATE_COUNT]
                           [VELOCITY_ESTIMATOR_STATE_COUNT] = {
                               {1.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0},
                               {0.0, 0.0, 1.0}};
  double transformed_covariance[VELOCITY_ESTIMATOR_STATE_COUNT]
                               [VELOCITY_ESTIMATOR_STATE_COUNT] = {{0.0}};
  double updated_covariance[VELOCITY_ESTIMATOR_STATE_COUNT]
                           [VELOCITY_ESTIMATOR_STATE_COUNT] = {{0.0}};

  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    residual_transform[row][VELOCITY_ESTIMATOR_STATE_ALTITUDE] -=
        kalman_gain[row];
  }

  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    for (uint8_t column = 0U; column < VELOCITY_ESTIMATOR_STATE_COUNT;
         column++)
    {
      for (uint8_t index = 0U; index < VELOCITY_ESTIMATOR_STATE_COUNT;
           index++)
      {
        transformed_covariance[row][column] +=
            residual_transform[row][index] *
            velocity_covariance[index][column];
      }
    }
  }

  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    for (uint8_t column = 0U; column < VELOCITY_ESTIMATOR_STATE_COUNT;
         column++)
    {
      for (uint8_t index = 0U; index < VELOCITY_ESTIMATOR_STATE_COUNT;
           index++)
      {
        updated_covariance[row][column] +=
            transformed_covariance[row][index] *
            residual_transform[column][index];
      }
      updated_covariance[row][column] +=
          kalman_gain[row] * measurement_variance * kalman_gain[column];
    }
  }

  uint8_t filter_state_is_finite = 1U;
  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    if (!isfinite(velocity_state[row]) ||
        !isfinite(updated_covariance[row][row]) ||
        updated_covariance[row][row] < 0.0)
    {
      filter_state_is_finite = 0U;
      break;
    }

    for (uint8_t column = 0U; column < VELOCITY_ESTIMATOR_STATE_COUNT;
         column++)
    {
      if (!isfinite(updated_covariance[row][column]))
      {
        filter_state_is_finite = 0U;
        break;
      }
    }
    if (!filter_state_is_finite)
    {
      break;
    }
  }

  if (!filter_state_is_finite)
  {
    VelocityEstimator_ResetSrf05Filter();
    VelocityEstimator_InitializeSrf05Filter(median_altitude_m,
                                            sample_time_ms);
    return 1U;
  }

  for (uint8_t row = 0U; row < VELOCITY_ESTIMATOR_STATE_COUNT; row++)
  {
    for (uint8_t column = 0U; column < VELOCITY_ESTIMATOR_STATE_COUNT;
         column++)
    {
      velocity_covariance[row][column] = updated_covariance[row][column];
    }
  }

  velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY] =
      VelocityEstimator_Clamp(
          velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY],
          -VELOCITY_ESTIMATOR_MAX_SPEED_MS,
          VELOCITY_ESTIMATOR_MAX_SPEED_MS);
  velocity_state[VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS] =
      VelocityEstimator_Clamp(
          velocity_state[VELOCITY_ESTIMATOR_STATE_ACCEL_BIAS],
          -VELOCITY_ESTIMATOR_MAX_RESIDUAL_BIAS_MS2,
          VELOCITY_ESTIMATOR_MAX_RESIDUAL_BIAS_MS2);

  srf05_filter_outliers = 0U;
  if (srf05_filter_inliers < UINT8_MAX)
  {
    srf05_filter_inliers++;
  }
  last_srf05_filter_update_ms = sample_time_ms;
  estimate.vertical_velocity_ms =
      velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY];

  uint8_t estimator_ready =
      srf05_filter_inliers >= VELOCITY_ESTIMATOR_SRF05_ACQUIRE_SAMPLES;
  if (SENSOR_MPU6050_ENABLED)
  {
    estimator_ready =
        estimator_ready && vertical_acceleration_bias_initialized;
  }
  estimate.vertical_velocity_valid = estimator_ready;
  return 1U;
}

void VelocityEstimator_Predict(double dt, uint32_t now_ms)
{
  if (SENSOR_SRF05_ENABLED)
  {
    VelocityEstimator_PredictSrf05(dt, now_ms);
  }
  else if (SENSOR_MPU6050_ENABLED)
  {
    double acceleration_ms2 = vertical_acceleration_ms2;
    if (fabs(acceleration_ms2) < VELOCITY_ESTIMATOR_ACCEL_DEADBAND_MS2)
    {
      acceleration_ms2 = 0.0;
    }
    acceleration_ms2 = VelocityEstimator_Clamp(
        acceleration_ms2, -VELOCITY_ESTIMATOR_MAX_ACCEL_MS2,
        VELOCITY_ESTIMATOR_MAX_ACCEL_MS2);

    estimate.vertical_velocity_ms += acceleration_ms2 * dt;
    estimate.vertical_velocity_ms = VelocityEstimator_Clamp(
        estimate.vertical_velocity_ms, -VELOCITY_ESTIMATOR_MAX_SPEED_MS,
        VELOCITY_ESTIMATOR_MAX_SPEED_MS);

    if (!SENSOR_BMP280_ENABLED)
    {
      estimate.vertical_velocity_valid =
          vertical_acceleration_bias_initialized;
    }
  }
  else
  {
    (void)dt;
  }

  if (SENSOR_BMP280_ENABLED && !SENSOR_SRF05_ENABLED)
  {
    if (have_previous_vertical_altitude &&
        (now_ms - previous_vertical_altitude_ms) >
            VELOCITY_ESTIMATOR_ALTITUDE_MAX_GAP_MS)
    {
      estimate.vertical_velocity_valid = 0U;
    }
  }
  else
  {
    (void)now_ms;
  }
}

VelocityEstimatorSrf05Result_t VelocityEstimator_CorrectSrf05Altitude(
    double altitude_m, uint32_t sample_time_ms)
{
  if (!SENSOR_SRF05_ENABLED)
  {
    (void)altitude_m;
    (void)sample_time_ms;
    return VELOCITY_ESTIMATOR_SRF05_REJECTED;
  }

  double vertical_altitude_m = altitude_m;
  if (SENSOR_MPU6050_ENABLED)
  {
    if (vertical_tilt_scale < VELOCITY_ESTIMATOR_SRF05_MIN_TILT_SCALE)
    {
      estimate.srf05_altitude_valid = 0U;
      estimate.vertical_velocity_valid = 0U;
      return VELOCITY_ESTIMATOR_SRF05_TILT_REJECTED;
    }
    vertical_altitude_m *= vertical_tilt_scale;
  }

  if (!VelocityEstimator_UpdateSrf05Filter(vertical_altitude_m,
                                           sample_time_ms))
  {
    return VELOCITY_ESTIMATOR_SRF05_REJECTED;
  }

  estimate.srf05_altitude_valid = 1U;
  estimate.srf05_altitude_cm =
      velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE] * 100.0;
  return VELOCITY_ESTIMATOR_SRF05_ACCEPTED;
}

void VelocityEstimator_HandleSrf05Timeout(void)
{
  if (SENSOR_SRF05_ENABLED)
  {
    estimate.srf05_altitude_valid = 0U;
    estimate.vertical_velocity_valid = 0U;
    VelocityEstimator_ResetSrf05Filter();
  }
}

void VelocityEstimator_CorrectBmp280Altitude(double altitude_m,
                                             uint32_t sample_time_ms)
{
  if (!SENSOR_BMP280_ENABLED || SENSOR_SRF05_ENABLED)
  {
    (void)altitude_m;
    (void)sample_time_ms;
    return;
  }

  if ((sample_time_ms - last_bmp280_sample_ms) <
      VELOCITY_ESTIMATOR_BMP280_SAMPLE_MS)
  {
    return;
  }
  last_bmp280_sample_ms = sample_time_ms;

  if (!isfinite(altitude_m))
  {
    return;
  }

  if (!have_previous_vertical_altitude)
  {
    previous_vertical_altitude_m = altitude_m;
    previous_vertical_altitude_ms = sample_time_ms;
    filtered_altitude_velocity_ms = estimate.vertical_velocity_ms;
    have_previous_vertical_altitude = 1U;
    estimate.vertical_velocity_valid = 0U;
    return;
  }

  const uint32_t elapsed_ms =
      sample_time_ms - previous_vertical_altitude_ms;
  const double altitude_change_m =
      altitude_m - previous_vertical_altitude_m;
  previous_vertical_altitude_m = altitude_m;
  previous_vertical_altitude_ms = sample_time_ms;

  if (elapsed_ms == 0U ||
      elapsed_ms > VELOCITY_ESTIMATOR_ALTITUDE_MAX_GAP_MS)
  {
    filtered_altitude_velocity_ms = estimate.vertical_velocity_ms;
    estimate.vertical_velocity_valid = 0U;
    return;
  }

  const double altitude_dt = (double)elapsed_ms / 1000.0;
  const double measured_altitude_velocity_ms =
      altitude_change_m / altitude_dt;
  if (!isfinite(measured_altitude_velocity_ms) ||
      fabs(measured_altitude_velocity_ms) >
          VELOCITY_ESTIMATOR_MAX_ALTITUDE_SPEED_MS)
  {
    return;
  }

  const double filter_gain =
      altitude_dt /
      (VELOCITY_ESTIMATOR_ALTITUDE_FILTER_TAU_S + altitude_dt);
  filtered_altitude_velocity_ms +=
      filter_gain *
      (measured_altitude_velocity_ms - filtered_altitude_velocity_ms);

  if (SENSOR_MPU6050_ENABLED)
  {
    const double fusion_gain =
        altitude_dt / (VELOCITY_ESTIMATOR_FUSION_TAU_S + altitude_dt);
    estimate.vertical_velocity_ms +=
        fusion_gain *
        (filtered_altitude_velocity_ms - estimate.vertical_velocity_ms);
  }
  else
  {
    estimate.vertical_velocity_ms = filtered_altitude_velocity_ms;
  }

  estimate.vertical_velocity_ms = VelocityEstimator_Clamp(
      estimate.vertical_velocity_ms, -VELOCITY_ESTIMATOR_MAX_SPEED_MS,
      VELOCITY_ESTIMATOR_MAX_SPEED_MS);
  estimate.vertical_velocity_valid = 1U;
}

VelocityEstimatorEstimate_t VelocityEstimator_GetEstimate(void)
{
  return estimate;
}
