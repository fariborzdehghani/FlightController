#include "SensorAcquisition.h"

#include "BMPXX80.h"
#include "MPU6050.h"
#include "QMC5883.h"
#include "SensorConfig.h"
#include "Tools.h"
#include "VelocityEstimator.h"
#include "srf05.h"
#include <math.h>

#define SENSOR_ACQUISITION_SRF05_SAMPLE_MS 50U
#define SENSOR_ACQUISITION_SRF05_MAX_GAP_MS 300U
#define SENSOR_ACQUISITION_YAW_STABLE_THRESHOLD_DEG 1.0
#define SENSOR_ACQUISITION_YAW_STABLE_SAMPLES 5U
#define SENSOR_ACQUISITION_YAW_TIMEOUT_MS 2000U
#define SENSOR_ACQUISITION_PI 3.14159265358979323846

static FlightData_t sensor_data;

static uint8_t srf05_measurement_pending;
static uint8_t srf05_have_success;
static uint8_t srf05_unavailable_reported;
static uint32_t last_srf05_attempt_ms;
static uint32_t last_srf05_success_ms;
static uint32_t srf05_monitor_started_ms;

static SensorAcquisitionResult_t SensorAcquisition_Result(
    SensorAcquisitionError_t error, HAL_StatusTypeDef hal_status)
{
  const SensorAcquisitionResult_t result = {
      .error = error, .hal_status = hal_status};
  return result;
}

static void SensorAcquisition_ApplyVelocityEstimate(void)
{
  const VelocityEstimatorEstimate_t estimate =
      VelocityEstimator_GetEstimate();
  sensor_data.Vz = estimate.vertical_velocity_ms;
  sensor_data.Vz_valid = estimate.vertical_velocity_valid;
  sensor_data.takeoff_altitude = estimate.srf05_altitude_cm;
  sensor_data.takeoff_altitude_valid = estimate.srf05_altitude_valid;
}

static void SensorAcquisition_ResetSrf05(void)
{
  srf05_measurement_pending = 0U;
  srf05_have_success = 0U;
  srf05_unavailable_reported = 0U;
  last_srf05_attempt_ms = 0U;
  last_srf05_success_ms = 0U;
  srf05_monitor_started_ms = 0U;
}

static void SensorAcquisition_UpdateSrf05Availability(uint32_t now_ms)
{
  const uint8_t measurement_is_stale =
      srf05_have_success
          ? ((now_ms - last_srf05_success_ms) >
             SENSOR_ACQUISITION_SRF05_MAX_GAP_MS)
          : ((now_ms - srf05_monitor_started_ms) >
             SENSOR_ACQUISITION_SRF05_MAX_GAP_MS);

  if (!measurement_is_stale || srf05_unavailable_reported)
  {
    return;
  }

  VelocityEstimator_HandleSrf05Timeout();
  SensorAcquisition_ApplyVelocityEstimate();
  srf05_have_success = 0U;
  srf05_monitor_started_ms = now_ms;
  LogError(2006, "SRF05 unavailable: altitude and Vz disabled");
  srf05_unavailable_reported = 1U;
}

static SensorAcquisitionResult_t SensorAcquisition_UpdateAttitude(void)
{
  if (SENSOR_MPU6050_ENABLED)
  {
    static MPU6050_t mpu_data;
    const HAL_StatusTypeDef mpu_status = MPU6050_ReadAll(&mpu_data);
    if (mpu_status != HAL_OK)
    {
      return SensorAcquisition_Result(
          SENSOR_ACQUISITION_ERROR_MPU6050_READ, mpu_status);
    }

    sensor_data.roll_deg = mpu_data.KalmanAngleY;
    sensor_data.pitch_deg = mpu_data.KalmanAngleX;
    sensor_data.Gz =
        VelocityEstimator_CorrectGyroZ(mpu_data.Gz);

    const VelocityEstimatorImuSample_t imu_sample = {
        .mpu_angle_x_deg = mpu_data.KalmanAngleX,
        .mpu_angle_y_deg = mpu_data.KalmanAngleY,
        .acceleration_x_ms2 = mpu_data.Ax_ms,
        .acceleration_y_ms2 = mpu_data.Ay_ms,
        .acceleration_z_ms2 = mpu_data.Az_ms,
        .gyro_x_dps = mpu_data.Gx,
        .gyro_y_dps = mpu_data.Gy,
        .gyro_z_dps = mpu_data.Gz};
    VelocityEstimator_UpdateImu(&imu_sample);
  }
  else
  {
    sensor_data.roll_deg = 0.0;
    sensor_data.pitch_deg = 0.0;
    sensor_data.Gz = 0.0;
  }

  if (SENSOR_QMC5883_ENABLED)
  {
    float yaw_deg;
    const HAL_StatusTypeDef qmc_status =
        QMC5883P_CompensatedYaw((float)sensor_data.roll_deg,
                                (float)sensor_data.pitch_deg, &yaw_deg);
    if (qmc_status != HAL_OK)
    {
      return SensorAcquisition_Result(
          SENSOR_ACQUISITION_ERROR_QMC5883_READ, qmc_status);
    }
    sensor_data.yaw_deg = yaw_deg;
  }
  else
  {
    sensor_data.yaw_deg = 0.0;
  }

  sensor_data.roll_rad =
      sensor_data.roll_deg * SENSOR_ACQUISITION_PI / 180.0;
  sensor_data.pitch_rad =
      sensor_data.pitch_deg * SENSOR_ACQUISITION_PI / 180.0;
  sensor_data.yaw_rad =
      sensor_data.yaw_deg * SENSOR_ACQUISITION_PI / 180.0;

  return SensorAcquisition_Result(SENSOR_ACQUISITION_ERROR_NONE, HAL_OK);
}

static SensorAcquisitionResult_t SensorAcquisition_WaitForYawStable(void)
{
  double last_yaw_deg = 0.0;
  uint8_t have_last_yaw = 0U;
  uint8_t stable_count = 0U;
  const uint32_t started_ms = HAL_GetTick();

  while (stable_count < SENSOR_ACQUISITION_YAW_STABLE_SAMPLES)
  {
    const SensorAcquisitionResult_t result =
        SensorAcquisition_UpdateAttitude();
    if (result.hal_status != HAL_OK)
    {
      return result;
    }

    if (have_last_yaw)
    {
      double yaw_change_deg =
          fabs(sensor_data.yaw_deg - last_yaw_deg);
      if (yaw_change_deg > 180.0)
      {
        yaw_change_deg = 360.0 - yaw_change_deg;
      }

      if (yaw_change_deg < SENSOR_ACQUISITION_YAW_STABLE_THRESHOLD_DEG)
      {
        stable_count++;
      }
      else
      {
        stable_count = 0U;
      }
    }

    last_yaw_deg = sensor_data.yaw_deg;
    have_last_yaw = 1U;
    HAL_Delay(10U);

    if ((HAL_GetTick() - started_ms) >=
        SENSOR_ACQUISITION_YAW_TIMEOUT_MS)
    {
      return SensorAcquisition_Result(
          SENSOR_ACQUISITION_ERROR_YAW_STABILIZATION_TIMEOUT,
          HAL_TIMEOUT);
    }
  }

  return SensorAcquisition_Result(SENSOR_ACQUISITION_ERROR_NONE, HAL_OK);
}

void SensorAcquisition_Init(void)
{
  sensor_data.roll_deg = 0.0;
  sensor_data.pitch_deg = 0.0;
  sensor_data.yaw_deg = 0.0;
  sensor_data.roll_rad = 0.0;
  sensor_data.pitch_rad = 0.0;
  sensor_data.yaw_rad = 0.0;
  sensor_data.Gz = 0.0;
  sensor_data.Vz = 0.0;
  sensor_data.Vz_valid = 0U;
  sensor_data.fly_altitude = 0.0;
  sensor_data.takeoff_altitude = 0.0;
  sensor_data.takeoff_altitude_valid = 0U;

  VelocityEstimator_Init();
  SensorAcquisition_ApplyVelocityEstimate();

  if (SENSOR_SRF05_ENABLED)
  {
    SRF05_CancelMeasurement();
    SensorAcquisition_ResetSrf05();
  }
}

SensorAcquisitionResult_t SensorAcquisition_Start(void)
{
  if (SENSOR_MPU6050_ENABLED)
  {
    for (uint8_t sample = 0U;
         sample < VELOCITY_ESTIMATOR_ACCEL_BIAS_SAMPLE_COUNT; sample++)
    {
      const SensorAcquisitionResult_t result =
          SensorAcquisition_UpdateAttitude();
      if (result.hal_status != HAL_OK)
      {
        return result;
      }
      HAL_Delay(5U);
    }
  }

  if (SENSOR_QMC5883_ENABLED)
  {
    const SensorAcquisitionResult_t yaw_result =
        SensorAcquisition_WaitForYawStable();
    if (yaw_result.hal_status != HAL_OK)
    {
      return yaw_result;
    }
  }

  if (SENSOR_SRF05_ENABLED)
  {
    const uint32_t started_ms = HAL_GetTick();
    last_srf05_attempt_ms =
        started_ms - SENSOR_ACQUISITION_SRF05_SAMPLE_MS;
    srf05_monitor_started_ms = started_ms;
  }

  return SensorAcquisition_Result(SENSOR_ACQUISITION_ERROR_NONE, HAL_OK);
}

SensorAcquisitionResult_t SensorAcquisition_Update(double dt)
{
  SensorAcquisitionResult_t result = SensorAcquisition_UpdateAttitude();
  if (result.hal_status != HAL_OK)
  {
    return result;
  }

  const uint32_t now_ms = HAL_GetTick();
  VelocityEstimator_Predict(dt, now_ms);
  SensorAcquisition_ApplyVelocityEstimate();

  if (SENSOR_SRF05_ENABLED)
  {
    if (srf05_measurement_pending)
    {
      float takeoff_altitude_cm;
      uint32_t sample_time_ms;
      const HAL_StatusTypeDef read_status =
          SRF05_ReadMeasurement(&takeoff_altitude_cm, &sample_time_ms);
      if (read_status == HAL_OK)
      {
        srf05_measurement_pending = 0U;
        const VelocityEstimatorSrf05Result_t measurement_result =
            VelocityEstimator_CorrectSrf05Altitude(
                (double)takeoff_altitude_cm * 0.01, sample_time_ms);
        SensorAcquisition_ApplyVelocityEstimate();

        if (measurement_result == VELOCITY_ESTIMATOR_SRF05_ACCEPTED)
        {
          srf05_have_success = 1U;
          last_srf05_success_ms = sample_time_ms;
          if (srf05_unavailable_reported)
          {
            LogInformation(
                1003,
                "SRF05 recovered: altitude measurements accepted");
            srf05_unavailable_reported = 0U;
          }
        }
      }
      else if (read_status != HAL_BUSY)
      {
        srf05_measurement_pending = 0U;
      }
    }

    const uint32_t srf05_now_ms = HAL_GetTick();
    if (!srf05_measurement_pending &&
        (srf05_now_ms - last_srf05_attempt_ms) >=
            SENSOR_ACQUISITION_SRF05_SAMPLE_MS)
    {
      if (SRF05_StartMeasurement() == HAL_OK)
      {
        srf05_measurement_pending = 1U;
        last_srf05_attempt_ms = srf05_now_ms;
      }
    }
    SensorAcquisition_UpdateSrf05Availability(srf05_now_ms);
  }
  else
  {
    sensor_data.takeoff_altitude = 0.0;
    sensor_data.takeoff_altitude_valid = 0U;
  }

  if (SENSOR_BMP280_ENABLED)
  {
    float barometer_altitude_m;
    const HAL_StatusTypeDef bmp_status =
        BMP280_ReadAltitude(&barometer_altitude_m);
    if (bmp_status != HAL_OK)
    {
      return SensorAcquisition_Result(
          SENSOR_ACQUISITION_ERROR_BMP280_READ, bmp_status);
    }
    sensor_data.fly_altitude = barometer_altitude_m;

    if (!SENSOR_SRF05_ENABLED)
    {
      VelocityEstimator_CorrectBmp280Altitude(
          (double)barometer_altitude_m, now_ms);
      SensorAcquisition_ApplyVelocityEstimate();
    }
  }
  else
  {
    sensor_data.fly_altitude = 0.0;
  }

  return SensorAcquisition_Result(SENSOR_ACQUISITION_ERROR_NONE, HAL_OK);
}

void SensorAcquisition_Stop(void)
{
  if (SENSOR_SRF05_ENABLED)
  {
    SRF05_CancelMeasurement();
    SensorAcquisition_ResetSrf05();
  }
  VelocityEstimator_ResetVertical();
  SensorAcquisition_ApplyVelocityEstimate();
}

FlightData_t SensorAcquisition_GetFlightData(void)
{
  return sensor_data;
}
