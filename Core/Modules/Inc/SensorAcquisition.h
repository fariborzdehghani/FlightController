#ifndef SENSOR_ACQUISITION_H
#define SENSOR_ACQUISITION_H

#include "Core.h"

typedef enum
{
  SENSOR_ACQUISITION_ERROR_NONE = 0,
  SENSOR_ACQUISITION_ERROR_MPU6050_CALIBRATION,
  SENSOR_ACQUISITION_ERROR_MPU6050_READ,
  SENSOR_ACQUISITION_ERROR_QMC5883_READ,
  SENSOR_ACQUISITION_ERROR_YAW_STABILIZATION_TIMEOUT,
  SENSOR_ACQUISITION_ERROR_BMP280_READ
} SensorAcquisitionError_t;

typedef struct
{
  SensorAcquisitionError_t error;
  HAL_StatusTypeDef hal_status;
} SensorAcquisitionResult_t;

void SensorAcquisition_Init(void);
SensorAcquisitionResult_t SensorAcquisition_Start(void);
SensorAcquisitionResult_t SensorAcquisition_Update(double dt);
void SensorAcquisition_Stop(void);
uint8_t SensorAcquisition_IsReady(void);
FlightData_t SensorAcquisition_GetFlightData(void);

#endif /* SENSOR_ACQUISITION_H */
