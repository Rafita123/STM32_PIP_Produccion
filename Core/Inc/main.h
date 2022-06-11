/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define D01_Encoder_Pin GPIO_PIN_1
#define D01_Encoder_GPIO_Port GPIOA
#define D01_Encoder_EXTI_IRQn EXTI1_IRQn
#define D02_Encoder_Pin GPIO_PIN_2
#define D02_Encoder_GPIO_Port GPIOA
#define D02_Encoder_EXTI_IRQn EXTI2_IRQn
#define D03_Encoder_Pin GPIO_PIN_3
#define D03_Encoder_GPIO_Port GPIOA
#define D03_Encoder_EXTI_IRQn EXTI3_IRQn
#define D04_Encoder_Pin GPIO_PIN_4
#define D04_Encoder_GPIO_Port GPIOA
#define D04_Encoder_EXTI_IRQn EXTI4_IRQn
#define IN2_1_Pin GPIO_PIN_12
#define IN2_1_GPIO_Port GPIOB
#define IN2_2_Pin GPIO_PIN_13
#define IN2_2_GPIO_Port GPIOB
#define IN1_2_Pin GPIO_PIN_14
#define IN1_2_GPIO_Port GPIOB
#define IN1_1_Pin GPIO_PIN_15
#define IN1_1_GPIO_Port GPIOB
#define IN4_1_Pin GPIO_PIN_12
#define IN4_1_GPIO_Port GPIOA
#define IN4_2_Pin GPIO_PIN_15
#define IN4_2_GPIO_Port GPIOA
#define IN3_2_Pin GPIO_PIN_3
#define IN3_2_GPIO_Port GPIOB
#define IN3_1_Pin GPIO_PIN_4
#define IN3_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
