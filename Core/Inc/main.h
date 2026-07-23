/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STATE_LED_Pin GPIO_PIN_6
#define STATE_LED_GPIO_Port GPIOA
#define SRF05_ECHO_Pin GPIO_PIN_0
#define SRF05_ECHO_GPIO_Port GPIOB
#define SRF05_TRIG_Pin GPIO_PIN_1
#define SRF05_TRIG_GPIO_Port GPIOB
#define Motor4_Pin GPIO_PIN_9
#define Motor4_GPIO_Port GPIOE
#define HMC5883L_DRY_Pin GPIO_PIN_14
#define HMC5883L_DRY_GPIO_Port GPIOE
#define Motor3_Pin GPIO_PIN_12
#define Motor3_GPIO_Port GPIOD
#define SX_RESET_Pin GPIO_PIN_6
#define SX_RESET_GPIO_Port GPIOC
#define SX_NSS_Pin GPIO_PIN_7
#define SX_NSS_GPIO_Port GPIOC
#define SX_IRQ_Pin GPIO_PIN_8
#define SX_IRQ_GPIO_Port GPIOC
#define SX_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define Motor2_Pin GPIO_PIN_15
#define Motor2_GPIO_Port GPIOA
#define HMC5883L_DRDY_Pin GPIO_PIN_0
#define HMC5883L_DRDY_GPIO_Port GPIOD
#define Motor1_Pin GPIO_PIN_4
#define Motor1_GPIO_Port GPIOB
#define MPU6050_INT_Pin GPIO_PIN_5
#define MPU6050_INT_GPIO_Port GPIOB
#define MPU6050_INT_EXTI_IRQn EXTI9_5_IRQn
#define MPU6050_SCL_Pin GPIO_PIN_6
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_7
#define MPU6050_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
