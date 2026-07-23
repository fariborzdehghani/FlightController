#ifndef __MotorControl_H
#define __MotorControl_H

#include "main.h"
#include "Core.h"

HAL_StatusTypeDef Motors_Init(TIM_HandleTypeDef *motor1_timer,
                             TIM_HandleTypeDef *motor2_timer,
                             TIM_HandleTypeDef *motor3_timer,
                             TIM_HandleTypeDef *motor4_timer);
void Motors_StartCalibration(void);
void Motors_FinishCalibration(void);
HAL_StatusTypeDef Motors_SetSpeed(MotorSpeeds_t speeds);
HAL_StatusTypeDef Motors_UpdateDutyCycle(TIM_HandleTypeDef *htim, double speed);
void Motors_Reset(MotorSpeeds_t* speeds);

#endif /* __MotorControl_H */
