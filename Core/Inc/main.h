/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C1_INT_Pin GPIO_PIN_1
#define I2C1_INT_GPIO_Port GPIOA
#define SPI1_CSN_Pin GPIO_PIN_0
#define SPI1_CSN_GPIO_Port GPIOB
#define SPI1_CE_Pin GPIO_PIN_1
#define SPI1_CE_GPIO_Port GPIOB
#define SPI1_IRQ_Pin GPIO_PIN_2
#define SPI1_IRQ_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LOOP_TIME             0.01

extern double Motors_Speed[4];

extern PID_t pid_roll;
extern PID_t pid_pitch;
extern PID_t pid_yaw;
extern PID_t pid_Vz;

extern float Config_BaseSpeed;
extern float Config_MinSpeed;
extern float Config_MaxSpeed;
extern float Config_MaxAngle;
extern float Config_TargetPitch;
extern float Config_TargetRoll;
extern float Config_TargetYaw;
extern float Config_TargetVz;
extern float Config_PitchKp;
extern float Config_Pitchki;
extern float Config_PitchKd;
extern float Config_RollKp;
extern float Config_RollKi;
extern float Config_RollKd;
extern float Config_YawKp;
extern float Config_YawKi;
extern float Config_YawKd;
extern float Config_VzKp;
extern float Config_VzKi;
extern float Config_VzKd;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
