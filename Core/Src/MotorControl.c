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

	// Set Max Speed using constant
	Motors_UpdateDutyCicle(Motor1, MOTOR_ABSOLUTE_MAX_SPEED);
	Motors_UpdateDutyCicle(Motor2, MOTOR_ABSOLUTE_MAX_SPEED);
	Motors_UpdateDutyCicle(Motor3, MOTOR_ABSOLUTE_MAX_SPEED);
	Motors_UpdateDutyCicle(Motor4, MOTOR_ABSOLUTE_MAX_SPEED);
	HAL_Delay(2000);

	// Set Min Speed using constant
	Motors_UpdateDutyCicle(Motor1, MOTOR_ABSOLUTE_MIN_SPEED);
	Motors_UpdateDutyCicle(Motor2, MOTOR_ABSOLUTE_MIN_SPEED);
	Motors_UpdateDutyCicle(Motor3, MOTOR_ABSOLUTE_MIN_SPEED);
	Motors_UpdateDutyCicle(Motor4, MOTOR_ABSOLUTE_MIN_SPEED);
	HAL_Delay(2000);

	LogInformation(1001, "Motors Calibrated!");

}

void Motors_SetSpeed(MotorSpeeds_t speeds)
{
    Motors_UpdateDutyCicle(Motor1, speeds.front_left);
    Motors_UpdateDutyCicle(Motor2, speeds.front_right);
    Motors_UpdateDutyCicle(Motor3, speeds.back_left);
    Motors_UpdateDutyCicle(Motor4, speeds.back_right);
}

void Motors_ClampSpeeds(MotorSpeeds_t* speeds, float minSpeed, float maxSpeed) {
    // Ensure speeds are within absolute limits first
    if (minSpeed < MOTOR_ABSOLUTE_MIN_SPEED) minSpeed = MOTOR_ABSOLUTE_MIN_SPEED;
    if (maxSpeed > MOTOR_ABSOLUTE_MAX_SPEED) maxSpeed = MOTOR_ABSOLUTE_MAX_SPEED;

    // Clamp all motor speeds
    speeds->front_left = (speeds->front_left < minSpeed) ? minSpeed : 
                        (speeds->front_left > maxSpeed) ? maxSpeed : speeds->front_left;
    speeds->front_right = (speeds->front_right < minSpeed) ? minSpeed : 
                         (speeds->front_right > maxSpeed) ? maxSpeed : speeds->front_right;
    speeds->back_left = (speeds->back_left < minSpeed) ? minSpeed : 
                       (speeds->back_left > maxSpeed) ? maxSpeed : speeds->back_left;
    speeds->back_right = (speeds->back_right < minSpeed) ? minSpeed : 
                        (speeds->back_right > maxSpeed) ? maxSpeed : speeds->back_right;
}

void Motors_Reset(MotorSpeeds_t* speeds) {
    speeds->front_left = 0;
    speeds->front_right = 0;
    speeds->back_left = 0;
    speeds->back_right = 0;
}

void Motors_UpdateDutyCicle(TIM_HandleTypeDef *htim, double Speed)
{
    // Add safety check
    if (Speed < MOTOR_ABSOLUTE_MIN_SPEED || Speed > MOTOR_ABSOLUTE_MAX_SPEED) {
        return;
    }
    
	uint32_t DutyCycle = 1000 + (Speed / 100) * 1000;
	if (DutyCycle > 2000)
	{
		DutyCycle = 2000;
	}
	__HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, DutyCycle);
}