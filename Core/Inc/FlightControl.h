#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#include "main.h"
#include "Core.h"

// Function declarations
HAL_StatusTypeDef FlightControl_Init(void);
HAL_StatusTypeDef FlightControl_Update(double dt);
void FlightControl_BeginTakeoff(void);
/* Live collective takeover; only +/-1 is accepted, within configured limits. */
uint8_t FlightControl_AdjustThrottle(int8_t direction);
uint8_t FlightControl_IsManualThrottle(void);
/* Last flight estimate is retained after stopping, and reset on a new start. */
double FlightControl_GetHoverThrottleEstimate(void);
uint8_t FlightControl_IsTakeoffComplete(void);
uint8_t FlightControl_IsLanding(void);
/* Unconfirmed takeoff is holding its base throttle to reacquire fresh height. */
uint8_t FlightControl_IsTakeoffRecovery(void);
void FlightControl_RequestLanding(void);
uint8_t FlightControl_IsLandingComplete(void);
void FlightControl_Stop(void);
HAL_StatusTypeDef FlightControl_Start(void);
void FlightControl_DesaturateSpeeds(MotorSpeeds_t *speeds, float minThrottle,
                                    float maxThrottle);
HAL_StatusTypeDef FlightControl_UpdateFlightData(double dt);

#endif /* __FLIGHT_CONTROL_H */
