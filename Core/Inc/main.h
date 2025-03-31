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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOT2_MS2_Pin GPIO_PIN_4
#define MOT2_MS2_GPIO_Port GPIOA
#define MOT2_MS3_Pin GPIO_PIN_5
#define MOT2_MS3_GPIO_Port GPIOA
#define MOT2_STEP_Pin GPIO_PIN_6
#define MOT2_STEP_GPIO_Port GPIOA
#define MOT2_DIR_Pin GPIO_PIN_7
#define MOT2_DIR_GPIO_Port GPIOA
#define MOT1_DIR_Pin GPIO_PIN_15
#define MOT1_DIR_GPIO_Port GPIOB
#define MOT1_STEP_Pin GPIO_PIN_8
#define MOT1_STEP_GPIO_Port GPIOA
#define MOT1_MS3_Pin GPIO_PIN_9
#define MOT1_MS3_GPIO_Port GPIOA
#define MOT1_MS2_Pin GPIO_PIN_10
#define MOT1_MS2_GPIO_Port GPIOA
#define MOT1_MS1_Pin GPIO_PIN_11
#define MOT1_MS1_GPIO_Port GPIOA
#define ICM20948_INT_Pin GPIO_PIN_6
#define ICM20948_INT_GPIO_Port GPIOB
#define ICM20948_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
