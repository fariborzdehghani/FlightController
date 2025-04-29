#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#include "main.h"
#include "Core.h"

// Function declarations
void FlightControl_Init(void);
void FlightControl_Update(double dt);
void FlightControl_Stop(void);
void FlightControl_Start(void);

#endif /* __FLIGHT_CONTROL_H */
