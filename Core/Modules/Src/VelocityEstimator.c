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
#define VELOCITY_ESTIMATOR_SRF05_MEASUREMENT_NOISE_M 0.01
#define VELOCITY_ESTIMATOR_SRF05_MIN_INNOVATION_GATE_M 0.08
#define VELOCITY_ESTIMATOR_SRF05_INNOVATION_SIGMA 5.0
#define VELOCITY_ESTIMATOR_ACCEL_PROCESS_NOISE_MS2 0.8
#define VELOCITY_ESTIMATOR_SRF05_RECOVERY_SAMPLES 3U
#define VELOCITY_ESTIMATOR_SRF05_RECOVERY_MAX_MOTION_M 0.12
#define VELOCITY_ESTIMATOR_SRF05_REACQUIRE_SAMPLES 5U
#define VELOCITY_ESTIMATOR_SRF05_REACQUIRE_SPAN_MS 250U
#define VELOCITY_ESTIMATOR_SRF05_REACQUIRE_MAX_SPAN_MS 500U
#define VELOCITY_ESTIMATOR_SRF05_REACQUIRE_MAX_OUTLIERS 2U
#define VELOCITY_ESTIMATOR_SRF05_REACQUIRE_MAX_SPEED_MS 1.0
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
static uint32_t last_srf05_filter_update_ms;
static uint32_t last_srf05_raw_sample_ms;
static uint8_t srf05_have_raw_sample;
static double last_srf05_accepted_altitude_m;
static double last_srf05_accepted_velocity_ms;
static double srf05_recovery_altitude_m;
static uint32_t srf05_recovery_sample_ms;
static uint8_t srf05_recovery_samples;
static uint8_t srf05_reacquisition_allowed;
static struct
{
  double first_altitude_m;
  double altitude_m;
  double velocity_ms;
  uint32_t first_ms;
  uint32_t last_ms;
  uint8_t samples;
  uint8_t outliers;
} srf05_reacquisition;

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
  last_srf05_filter_update_ms = 0U;
  last_srf05_raw_sample_ms = 0U;
  srf05_have_raw_sample = 0U;
  last_srf05_accepted_altitude_m = 0.0;
  last_srf05_accepted_velocity_ms = 0.0;
  srf05_recovery_samples = 0U;
  srf05_reacquisition.samples = 0U;
  estimate.vertical_velocity_ms = 0.0;
  estimate.vertical_velocity_valid = 0U;
  estimate.srf05_altitude_valid = 0U;
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
  srf05_reacquisition_allowed = 0U;
  if (SENSOR_MPU6050_ENABLED)
  {
    VelocityEstimator_ResetGyroZ();
  }
  estimate.srf05_altitude_cm = 0.0;
  VelocityEstimator_ResetVertical();
}

void VelocityEstimator_SetSrf05ReacquisitionAllowed(uint8_t allowed)
{
  srf05_reacquisition_allowed = allowed != 0U;
  if (!srf05_reacquisition_allowed)
  {
    srf05_reacquisition.samples = 0U;
  }
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

static VelocityEstimatorSrf05Result_t VelocityEstimator_FilterSrf05Altitude(
    double altitude_m, uint32_t sample_time_ms, double *filtered_altitude_m)
{
  if (filtered_altitude_m == NULL || !isfinite(altitude_m))
  {
    return VELOCITY_ESTIMATOR_SRF05_REJECTED;
  }

  uint32_t elapsed_ms = VELOCITY_ESTIMATOR_SRF05_SAMPLE_MS;
  if (srf05_have_raw_sample)
  {
    elapsed_ms = sample_time_ms - last_srf05_raw_sample_ms;
    if (elapsed_ms == 0U || elapsed_ms > INT32_MAX)
    {
      return VELOCITY_ESTIMATOR_SRF05_BAD_TIMESTAMP;
    }
    if (elapsed_ms > VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS)
    {
      srf05_median_count = 0U;
      srf05_median_index = 0U;
    }
  }
  last_srf05_raw_sample_ms = sample_time_ms;
  srf05_have_raw_sample = 1U;

  if (srf05_filter_initialized)
  {
    /* Tracking uses only the accepted echo history and prediction below.
     * Rejected raw spikes must not enter a median and veto the next real echo. */
    *filtered_altitude_m = altitude_m;
    return VELOCITY_ESTIMATOR_SRF05_ACCEPTED;
  }

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
    return VELOCITY_ESTIMATOR_SRF05_MEDIAN_WARMUP;
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

  /* Initial acquisition has no trusted prediction yet. Require a coherent
   * three-echo window instead of treating one member as a validated surface. */
  for (uint8_t i = 0U; i < VELOCITY_ESTIMATOR_SRF05_MEDIAN_SAMPLES; i++)
  {
    if (fabs(srf05_median_altitude_m[i] - *filtered_altitude_m) >
        VELOCITY_ESTIMATOR_SRF05_MEDIAN_GATE_M)
    {
      return VELOCITY_ESTIMATOR_SRF05_MOTION_REJECTED;
    }
  }

  *filtered_altitude_m = altitude_m;
  return VELOCITY_ESTIMATOR_SRF05_ACCEPTED;
}

static void VelocityEstimator_InitializeSrf05Filter(
    double altitude_m, uint32_t sample_time_ms)
{
  velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE] = altitude_m;
  velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY] = 0.0;

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
  srf05_filter_initialized = 1U;
  srf05_filter_inliers = 1U;
  last_srf05_filter_update_ms = sample_time_ms;
  last_srf05_accepted_altitude_m = altitude_m;
  last_srf05_accepted_velocity_ms = 0.0;
  srf05_recovery_samples = 0U;
  srf05_reacquisition.samples = 0U;
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
    /* Preserve the trusted reference. Recovery uses separate confirmation
     * gates; merely becoming stale must not validate a different surface. */
    estimate.srf05_altitude_valid = 0U;
    estimate.vertical_velocity_valid = 0U;
    return;
  }

  /* Height and speed follow accepted distance echoes. Motor vibration and
   * accelerometer offset must not integrate into a fictitious height, then
   * make the echo gate reject the real floor. IMU-only/BMP280 paths below
   * retain their existing acceleration fusion. */
  const double half_dt_squared = 0.5 * dt * dt;
  velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE] +=
      velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY] * dt;

  double transition[VELOCITY_ESTIMATOR_STATE_COUNT]
                   [VELOCITY_ESTIMATOR_STATE_COUNT] = {
                       {1.0, dt},
                       {0.0, 1.0}};

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
  if (!isfinite(velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE]) ||
      velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE] < 0.0)
  {
    /* A downward extrapolation cannot be a usable negative distance. Keep
     * the cached output invalid until a real echo establishes height again. */
    estimate.srf05_altitude_valid = 0U;
    estimate.vertical_velocity_valid = 0U;
    return;
  }
  estimate.vertical_velocity_ms =
      velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY];
  if (estimate.srf05_altitude_valid)
  {
    estimate.srf05_altitude_cm =
        velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE] * 100.0;
  }
}

static uint8_t VelocityEstimator_TryRecoverSrf05(
    double altitude_m, uint32_t sample_time_ms)
{
  /* Prediction can be wrong after motion or a dropout. Recover only near an
   * independently retained accepted echo; elapsed time never opens the gate
   * indefinitely toward a different reflecting surface. */
  const double age_s = fmin(
      (double)(sample_time_ms - last_srf05_filter_update_ms) / 1000.0,
      (double)VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS / 1000.0);
  const double speed_ms = fabs(last_srf05_accepted_velocity_ms);
  const double recovery_gate_m =
      VELOCITY_ESTIMATOR_SRF05_MIN_INNOVATION_GATE_M +
      fmin(speed_ms * age_s,
           VELOCITY_ESTIMATOR_SRF05_RECOVERY_MAX_MOTION_M);
  if (fabs(altitude_m - last_srf05_accepted_altitude_m) > recovery_gate_m)
  {
    srf05_recovery_samples = 0U;
    return 0U;
  }

  const uint32_t gap_ms = sample_time_ms - srf05_recovery_sample_ms;
  const double gap_s = (double)gap_ms / 1000.0;
  const double local_gate_m = VELOCITY_ESTIMATOR_SRF05_MEDIAN_GATE_M +
      speed_ms * gap_s + 0.5 * VELOCITY_ESTIMATOR_SRF05_MOTION_GATE_ACCEL_MS2 *
                              gap_s * gap_s;
  if (srf05_recovery_samples == 0U ||
      gap_ms > VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS ||
      fabs(altitude_m - srf05_recovery_altitude_m) > local_gate_m)
  {
    srf05_recovery_samples = 0U;
  }
  srf05_recovery_altitude_m = altitude_m;
  srf05_recovery_sample_ms = sample_time_ms;
  srf05_recovery_samples++;
  if (srf05_recovery_samples < VELOCITY_ESTIMATOR_SRF05_RECOVERY_SAMPLES)
  {
    return 0U;
  }

  VelocityEstimator_InitializeSrf05Filter(altitude_m, sample_time_ms);
  return 1U;
}

static void VelocityEstimator_StartSrf05Reacquisition(
    double altitude_m, uint32_t sample_time_ms)
{
  srf05_reacquisition.first_altitude_m = altitude_m;
  srf05_reacquisition.altitude_m = altitude_m;
  srf05_reacquisition.velocity_ms = 0.0;
  srf05_reacquisition.first_ms = sample_time_ms;
  srf05_reacquisition.last_ms = sample_time_ms;
  srf05_reacquisition.samples = 1U;
  srf05_reacquisition.outliers = 0U;
}

static uint8_t VelocityEstimator_TryReacquireSrf05(
    double altitude_m, uint32_t sample_time_ms)
{
  if (!srf05_reacquisition_allowed ||
      sample_time_ms - last_srf05_filter_update_ms <=
          VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS)
  {
    srf05_reacquisition.samples = 0U;
    return 0U;
  }

  const uint32_t gap_ms = sample_time_ms - srf05_reacquisition.last_ms;
  if (srf05_reacquisition.samples == 0U ||
      gap_ms > VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS ||
      sample_time_ms - srf05_reacquisition.first_ms >
          VELOCITY_ESTIMATOR_SRF05_REACQUIRE_MAX_SPAN_MS)
  {
    VelocityEstimator_StartSrf05Reacquisition(altitude_m, sample_time_ms);
    return 0U;
  }

  const double gap_s = (double)gap_ms / 1000.0;
  const double span_s =
      (double)(sample_time_ms - srf05_reacquisition.first_ms) / 1000.0;
  const double prediction_m = srf05_reacquisition.altitude_m +
      srf05_reacquisition.velocity_ms * gap_s;
  const double local_gate_m = VELOCITY_ESTIMATOR_SRF05_MEDIAN_GATE_M +
      0.5 * VELOCITY_ESTIMATOR_SRF05_MOTION_GATE_ACCEL_MS2 * gap_s * gap_s;
  if (fabs(altitude_m - prediction_m) > local_gate_m ||
      fabs(altitude_m - srf05_reacquisition.first_altitude_m) >
          VELOCITY_ESTIMATOR_SRF05_MEDIAN_GATE_M +
              VELOCITY_ESTIMATOR_SRF05_REACQUIRE_MAX_SPEED_MS * span_s)
  {
    /* Isolated spikes do not confirm or replace the candidate. Repeated
     * contradictions do: sparse minority matches cannot certify a surface. */
    srf05_reacquisition.outliers++;
    if (srf05_reacquisition.outliers >
        VELOCITY_ESTIMATOR_SRF05_REACQUIRE_MAX_OUTLIERS)
    {
      VelocityEstimator_StartSrf05Reacquisition(altitude_m, sample_time_ms);
    }
    return 0U;
  }

  srf05_reacquisition.velocity_ms = VelocityEstimator_Clamp(
      (altitude_m - srf05_reacquisition.first_altitude_m) / span_s,
      -VELOCITY_ESTIMATOR_SRF05_REACQUIRE_MAX_SPEED_MS,
      VELOCITY_ESTIMATOR_SRF05_REACQUIRE_MAX_SPEED_MS);
  srf05_reacquisition.altitude_m = altitude_m;
  srf05_reacquisition.last_ms = sample_time_ms;
  if (srf05_reacquisition.samples < VELOCITY_ESTIMATOR_SRF05_REACQUIRE_SAMPLES)
  {
    srf05_reacquisition.samples++;
  }
  if (srf05_reacquisition.samples < VELOCITY_ESTIMATOR_SRF05_REACQUIRE_SAMPLES ||
      sample_time_ms - srf05_reacquisition.first_ms <
          VELOCITY_ESTIMATOR_SRF05_REACQUIRE_SPAN_MS)
  {
    return 0U;
  }

  /* This is a confirmed alternative echo track, not proof of true height.
   * Core permits it while stopped, landing, or holding an unconfirmed takeoff
   * ramp. Reset speed conservatively; the controller confirms handover separately. */
  VelocityEstimator_InitializeSrf05Filter(altitude_m, sample_time_ms);
  return 1U;
}

static VelocityEstimatorSrf05Result_t VelocityEstimator_UpdateSrf05Filter(
    double altitude_m, uint32_t sample_time_ms)
{
  double median_altitude_m;
  const VelocityEstimatorSrf05Result_t result = VelocityEstimator_FilterSrf05Altitude(
      altitude_m, sample_time_ms, &median_altitude_m);
  if (result != VELOCITY_ESTIMATOR_SRF05_ACCEPTED)
  {
    srf05_recovery_samples = 0U;
    srf05_reacquisition.samples = 0U;
    return result;
  }

  if (!srf05_filter_initialized)
  {
    VelocityEstimator_InitializeSrf05Filter(median_altitude_m,
                                            sample_time_ms);
    return VELOCITY_ESTIMATOR_SRF05_ACCEPTED;
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
    return VELOCITY_ESTIMATOR_SRF05_INVALID_STATE;
  }

  /* The initial velocity covariance is deliberately broad. Until tracking is
   * established, it must not make a large jump look like a plausible echo. */
  const double innovation_gate =
      (srf05_filter_inliers < VELOCITY_ESTIMATOR_SRF05_ACQUIRE_SAMPLES)
          ? VELOCITY_ESTIMATOR_SRF05_MIN_INNOVATION_GATE_M
          : fmax(VELOCITY_ESTIMATOR_SRF05_MIN_INNOVATION_GATE_M,
                 VELOCITY_ESTIMATOR_SRF05_INNOVATION_SIGMA *
                     sqrt(innovation_variance));
  const double accepted_gap_s = fmin(
      (double)(sample_time_ms - last_srf05_filter_update_ms) / 1000.0,
      (double)VELOCITY_ESTIMATOR_SRF05_MAX_GAP_MS / 1000.0);
  const double motion_gate_m = VELOCITY_ESTIMATOR_SRF05_MEDIAN_GATE_M +
      fabs(last_srf05_accepted_velocity_ms) * accepted_gap_s +
      0.5 * VELOCITY_ESTIMATOR_SRF05_MOTION_GATE_ACCEL_MS2 *
          accepted_gap_s * accepted_gap_s;
  const uint8_t motion_rejected =
      fabs(median_altitude_m - last_srf05_accepted_altitude_m) > motion_gate_m;
  if (motion_rejected || fabs(innovation) > innovation_gate)
  {
    if (VelocityEstimator_TryRecoverSrf05(median_altitude_m, sample_time_ms) ||
        VelocityEstimator_TryReacquireSrf05(median_altitude_m, sample_time_ms))
    {
      return VELOCITY_ESTIMATOR_SRF05_ACCEPTED;
    }
    return motion_rejected ? VELOCITY_ESTIMATOR_SRF05_MOTION_REJECTED
                           : VELOCITY_ESTIMATOR_SRF05_INNOVATION_REJECTED;
  }
  srf05_recovery_samples = 0U;
  srf05_reacquisition.samples = 0U;

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
                               {1.0, 0.0},
                               {0.0, 1.0}};
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
    return 0U;
  }
  if (velocity_state[VELOCITY_ESTIMATOR_STATE_ALTITUDE] < 0.0)
  {
    /* This positive echo passed both gates, but a downward extrapolation
     * dominated the update. Restart from the measurement, never publish a
     * negative distance as valid. */
    VelocityEstimator_InitializeSrf05Filter(median_altitude_m, sample_time_ms);
    return VELOCITY_ESTIMATOR_SRF05_ACCEPTED;
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

  if (srf05_filter_inliers < UINT8_MAX)
  {
    srf05_filter_inliers++;
  }
  last_srf05_filter_update_ms = sample_time_ms;
  last_srf05_accepted_altitude_m = median_altitude_m;
  last_srf05_accepted_velocity_ms =
      velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY];
  estimate.vertical_velocity_ms =
      velocity_state[VELOCITY_ESTIMATOR_STATE_VELOCITY];

  const uint8_t estimator_ready =
      srf05_filter_inliers >= VELOCITY_ESTIMATOR_SRF05_ACQUIRE_SAMPLES;
  estimate.vertical_velocity_valid = estimator_ready;
  return VELOCITY_ESTIMATOR_SRF05_ACCEPTED;
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
  if (!isfinite(altitude_m) || altitude_m <= 0.0)
  {
    srf05_recovery_samples = 0U;
    srf05_reacquisition.samples = 0U;
    return VELOCITY_ESTIMATOR_SRF05_REJECTED;
  }

  double vertical_altitude_m = altitude_m;
  if (SENSOR_MPU6050_ENABLED)
  {
    if (vertical_tilt_scale < VELOCITY_ESTIMATOR_SRF05_MIN_TILT_SCALE)
    {
      srf05_recovery_samples = 0U;
      srf05_reacquisition.samples = 0U;
      estimate.srf05_altitude_valid = 0U;
      estimate.vertical_velocity_valid = 0U;
      return VELOCITY_ESTIMATOR_SRF05_TILT_REJECTED;
    }
    vertical_altitude_m *= vertical_tilt_scale;
  }

  const VelocityEstimatorSrf05Result_t result =
      VelocityEstimator_UpdateSrf05Filter(vertical_altitude_m, sample_time_ms);
  if (result != VELOCITY_ESTIMATOR_SRF05_ACCEPTED)
  {
    return result;
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
    /* A timeout alone preserves the reference. Wider reacquisition needs
     * separate phase permission and fresh coherent confirmation. */
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
