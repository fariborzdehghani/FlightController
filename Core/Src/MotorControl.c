#include "MotorControl.h"
#include "Tools.h"
#include "stm32f4xx_hal.h"
#include <math.h>

static TIM_HandleTypeDef *Motor1, *Motor2, *Motor3, *Motor4;

static uint32_t duty_cycle;

HAL_StatusTypeDef Motors_Init(TIM_HandleTypeDef *motor1_timer,
                              TIM_HandleTypeDef *motor2_timer,
                              TIM_HandleTypeDef *motor3_timer,
                              TIM_HandleTypeDef *motor4_timer) {
  if (motor1_timer == NULL || motor2_timer == NULL || motor3_timer == NULL ||
      motor4_timer == NULL) {
    return HAL_ERROR;
  }

  Motor1 = motor1_timer;
  Motor2 = motor2_timer;
  Motor3 = motor3_timer;
  Motor4 = motor4_timer;

  // Start PWM
  if (HAL_TIM_PWM_Start(Motor1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(Motor2, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(Motor3, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(Motor4, TIM_CHANNEL_1) != HAL_OK) {
    return HAL_ERROR;
  }

  // HAL_Delay(1000);
  // Motors_StartCalibration();
  // HAL_Delay(2000);
  // Motors_FinishCalibration();
  // HAL_Delay(2000);

  if (Motors_UpdateDutyCycle(Motor1, MOTOR_ABSOLUTE_MIN_THROTTLE) != HAL_OK ||
      Motors_UpdateDutyCycle(Motor2, MOTOR_ABSOLUTE_MIN_THROTTLE) != HAL_OK ||
      Motors_UpdateDutyCycle(Motor3, MOTOR_ABSOLUTE_MIN_THROTTLE) != HAL_OK ||
      Motors_UpdateDutyCycle(Motor4, MOTOR_ABSOLUTE_MIN_THROTTLE) != HAL_OK) {
    return HAL_ERROR;
  }

  LogInformation(1001, "Starting Initialized!");
  return HAL_OK;
}

void Motors_StartCalibration(void) {
  // Set maximum throttle using constant
  (void)Motors_UpdateDutyCycle(Motor1, MOTOR_ABSOLUTE_MAX_THROTTLE);
  (void)Motors_UpdateDutyCycle(Motor2, MOTOR_ABSOLUTE_MAX_THROTTLE);
  (void)Motors_UpdateDutyCycle(Motor3, MOTOR_ABSOLUTE_MAX_THROTTLE);
  (void)Motors_UpdateDutyCycle(Motor4, MOTOR_ABSOLUTE_MAX_THROTTLE);
}

void Motors_FinishCalibration(void) {
  // Set minimum throttle using constant
  (void)Motors_UpdateDutyCycle(Motor1, MOTOR_ABSOLUTE_MIN_THROTTLE);
  (void)Motors_UpdateDutyCycle(Motor2, MOTOR_ABSOLUTE_MIN_THROTTLE);
  (void)Motors_UpdateDutyCycle(Motor3, MOTOR_ABSOLUTE_MIN_THROTTLE);
  (void)Motors_UpdateDutyCycle(Motor4, MOTOR_ABSOLUTE_MIN_THROTTLE);

  LogInformation(1001, "Motors Calibrated!");
}

HAL_StatusTypeDef Motors_SetSpeed(MotorSpeeds_t speeds) {
  if (Motors_UpdateDutyCycle(Motor1, speeds.front_left) != HAL_OK ||
      Motors_UpdateDutyCycle(Motor2, speeds.front_right) != HAL_OK ||
      Motors_UpdateDutyCycle(Motor3, speeds.back_right) != HAL_OK ||
      Motors_UpdateDutyCycle(Motor4, speeds.back_left) != HAL_OK) {
    (void)Motors_UpdateDutyCycle(Motor1, MOTOR_ABSOLUTE_MIN_THROTTLE);
    (void)Motors_UpdateDutyCycle(Motor2, MOTOR_ABSOLUTE_MIN_THROTTLE);
    (void)Motors_UpdateDutyCycle(Motor3, MOTOR_ABSOLUTE_MIN_THROTTLE);
    (void)Motors_UpdateDutyCycle(Motor4, MOTOR_ABSOLUTE_MIN_THROTTLE);
    return HAL_ERROR;
  }
  return HAL_OK;
}

void Motors_Reset(MotorSpeeds_t *speeds) {
  speeds->front_left = 0;
  speeds->front_right = 0;
  speeds->back_left = 0;
  speeds->back_right = 0;
}

HAL_StatusTypeDef Motors_UpdateDutyCycle(TIM_HandleTypeDef *htim, double speed) {
  if (htim == NULL || !isfinite(speed) ||
      speed < MOTOR_ABSOLUTE_MIN_THROTTLE ||
      speed > MOTOR_ABSOLUTE_MAX_THROTTLE) {
    return HAL_ERROR;
  }

  duty_cycle = (uint32_t)(1000.0 + (speed / 100.0) * 1000.0);

  if (duty_cycle > 2000u) {
    duty_cycle = 2000u;
  }

  __HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, duty_cycle);
  return HAL_OK;
}
