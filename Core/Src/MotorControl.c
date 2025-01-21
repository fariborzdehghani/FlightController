#include "MotorControl.h"
#include "Tools.h"

TIM_HandleTypeDef *Motor1, *Motor2, *Motor3, *Motor4;

void Motors_Init(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4)
{
	LogInformation(1002, "Calibrating Motors...");
	Motor1 = htim1;
	Motor2 = htim2;
	Motor3 = htim3;
	Motor4 = htim4;

	// Start PWM
	HAL_TIM_PWM_Start(Motor1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(Motor2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(Motor3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(Motor4, TIM_CHANNEL_1);

	// Set Max Speed
	Motors_UpdateDutyCicle(Motor1, 100);
	Motors_UpdateDutyCicle(Motor2, 100);
	Motors_UpdateDutyCicle(Motor3, 100);
	Motors_UpdateDutyCicle(Motor4, 100);
	HAL_Delay(2000);

	// Set Min Speed
	Motors_UpdateDutyCicle(Motor1, 0);
	Motors_UpdateDutyCicle(Motor2, 0);
	Motors_UpdateDutyCicle(Motor3, 0);
	Motors_UpdateDutyCicle(Motor4, 0);
	HAL_Delay(2000);

	LogInformation(1001, "Motors Calibrated!");

}

void Motors_SetSpeed(double Motors_Speed[4])
{
	Motors_UpdateDutyCicle(Motor1, Motors_Speed[0]);
	Motors_UpdateDutyCicle(Motor2, Motors_Speed[1]);
	Motors_UpdateDutyCicle(Motor3, Motors_Speed[2]);
	Motors_UpdateDutyCicle(Motor4, Motors_Speed[3]);
}

void Motors_UpdateDutyCicle(TIM_HandleTypeDef *htim, double Speed)
{
	uint32_t DutyCycle = 1000 + (Speed / 100) * 1000;
	if (DutyCycle > 2000)
	{
		DutyCycle = 2000;
	}
	__HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, DutyCycle);
}