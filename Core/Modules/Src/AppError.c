#include "AppError.h"

static AppErrorCode_t last_error = APP_ERROR_NONE;
static HAL_StatusTypeDef last_hal_status = HAL_OK;

static const char *AppError_SensorMessage(
    const char *timeout_message, const char *busy_message,
    const char *error_message)
{
  if (last_hal_status == HAL_TIMEOUT)
  {
    return timeout_message;
  }
  if (last_hal_status == HAL_BUSY)
  {
    return busy_message;
  }
  return error_message;
}

void AppError_Clear(void)
{
  last_error = APP_ERROR_NONE;
  last_hal_status = HAL_OK;
}

HAL_StatusTypeDef AppError_Set(AppErrorCode_t code,
                               HAL_StatusTypeDef hal_status)
{
  last_error = code;
  last_hal_status = hal_status;
  return hal_status;
}

HAL_StatusTypeDef AppError_SetSensorAcquisitionResult(
    SensorAcquisitionResult_t result)
{
  if (result.hal_status == HAL_OK)
  {
    return HAL_OK;
  }

  switch (result.error)
  {
  case SENSOR_ACQUISITION_ERROR_MPU6050_CALIBRATION:
    return AppError_Set(APP_ERROR_MPU6050_CALIBRATION,
                        result.hal_status);
  case SENSOR_ACQUISITION_ERROR_MPU6050_READ:
    return AppError_Set(APP_ERROR_MPU6050_READ, result.hal_status);
  case SENSOR_ACQUISITION_ERROR_QMC5883_READ:
    return AppError_Set(APP_ERROR_QMC5883_READ, result.hal_status);
  case SENSOR_ACQUISITION_ERROR_YAW_STABILIZATION_TIMEOUT:
    return AppError_Set(APP_ERROR_YAW_STABILIZATION_TIMEOUT,
                        result.hal_status);
  case SENSOR_ACQUISITION_ERROR_BMP280_READ:
    return AppError_Set(APP_ERROR_BMP280_READ, result.hal_status);
  case SENSOR_ACQUISITION_ERROR_NONE:
  default:
    return AppError_Set(APP_ERROR_SENSOR_ACQUISITION,
                        result.hal_status);
  }
}

AppErrorCode_t AppError_GetCode(void)
{
  return last_error;
}

HAL_StatusTypeDef AppError_GetHalStatus(void)
{
  return last_hal_status;
}

const char *AppError_GetMessage(void)
{
  switch (last_error)
  {
  case APP_ERROR_INVALID_FLIGHT_TIMESTEP:
    return "Flight loop timing is invalid or exceeded 250 ms";
  case APP_ERROR_MPU6050_CALIBRATION:
    return AppError_SensorMessage(
        "MPU6050 calibration timed out: keep the aircraft stationary and retry",
        "MPU6050 calibration could not read sensor: I2C bus busy",
        "MPU6050 calibration failed");
  case APP_ERROR_MPU6050_READ:
    return AppError_SensorMessage(
        "MPU6050 attitude read timed out",
        "MPU6050 attitude read could not start: I2C bus busy",
        "MPU6050 attitude read failed");
  case APP_ERROR_QMC5883_READ:
    return AppError_SensorMessage(
        "QMC5883 yaw read timed out",
        "QMC5883 yaw read could not start: I2C bus busy",
        "QMC5883 yaw read failed");
  case APP_ERROR_YAW_STABILIZATION_TIMEOUT:
    return "Yaw did not stabilize within 2 seconds";
  case APP_ERROR_ARM_MOTOR_OUTPUT:
    return "Failed to command safe motor output while arming";
  case APP_ERROR_MAXIMUM_FLIGHT_ANGLE:
    return "Maximum configured roll or pitch angle exceeded";
  case APP_ERROR_MOTOR_OUTPUT:
    return "Motor output is invalid or could not be applied";
  case APP_ERROR_ALTITUDE_UNAVAILABLE:
    return "SRF05 altitude estimate was lost during flight";
  case APP_ERROR_BMP280_READ:
    return AppError_SensorMessage(
        "BMP280 altitude read timed out",
        "BMP280 altitude read could not start: I2C bus busy",
        "BMP280 altitude read failed");
  case APP_ERROR_SENSOR_ACQUISITION:
    return "Sensor acquisition failed";
  case APP_ERROR_NONE:
  default:
    return "No application error";
  }
}
