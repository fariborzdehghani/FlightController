#ifndef APP_ERROR_H
#define APP_ERROR_H

#include "main.h"
#include "SensorAcquisition.h"

typedef enum
{
  APP_ERROR_NONE = 0,
  APP_ERROR_INVALID_FLIGHT_TIMESTEP,
  APP_ERROR_MPU6050_READ,
  APP_ERROR_QMC5883_READ,
  APP_ERROR_YAW_STABILIZATION_TIMEOUT,
  APP_ERROR_ARM_MOTOR_OUTPUT,
  APP_ERROR_MAXIMUM_FLIGHT_ANGLE,
  APP_ERROR_MOTOR_OUTPUT,
  APP_ERROR_ALTITUDE_UNAVAILABLE,
  APP_ERROR_BMP280_READ,
  APP_ERROR_SENSOR_ACQUISITION
} AppErrorCode_t;

void AppError_Clear(void);
HAL_StatusTypeDef AppError_Set(AppErrorCode_t code,
                               HAL_StatusTypeDef hal_status);
HAL_StatusTypeDef AppError_SetSensorAcquisitionResult(
    SensorAcquisitionResult_t result);
AppErrorCode_t AppError_GetCode(void);
HAL_StatusTypeDef AppError_GetHalStatus(void);
const char *AppError_GetMessage(void);

#endif /* APP_ERROR_H */
