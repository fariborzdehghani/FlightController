#ifndef __MotorControl_H
#define __MotorControl_H

#include "main.h"

void Motors_Init(TIM_HandleTypeDef *htim1,TIM_HandleTypeDef *htim2,TIM_HandleTypeDef *htim3,TIM_HandleTypeDef *htim4);
void Motors_SetSpeed(double Motors_Speed[4]);
void Motors_UpdateDutyCicle(TIM_HandleTypeDef *htim, double Speed);

#endif /* __MotorControl_H */