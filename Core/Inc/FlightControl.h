#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#include "main.h"
#include "Core.h"

// Function declarations
HAL_StatusTypeDef FlightControl_Init(void);
HAL_StatusTypeDef FlightControl_Update(double dt);
void FlightControl_Stop(void);
HAL_StatusTypeDef FlightControl_Start(void);
void FlightControl_ClampSpeeds(MotorSpeeds_t* speeds, float minSpeed, float maxSpeed);
HAL_StatusTypeDef FlightControl_UpdateFlightData(double dt);

#endif /* __FLIGHT_CONTROL_H */
