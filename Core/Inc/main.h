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
#define LED_A4_Pin GPIO_PIN_4
#define LED_A4_GPIO_Port GPIOA
#define LED_A5_Pin GPIO_PIN_5
#define LED_A5_GPIO_Port GPIOA
#define LED_A6_Pin GPIO_PIN_6
#define LED_A6_GPIO_Port GPIOA
#define TIM1_CH1N_E8_Pin GPIO_PIN_8
#define TIM1_CH1N_E8_GPIO_Port GPIOE
#define Boton_S1_IT_Pin GPIO_PIN_10
#define Boton_S1_IT_GPIO_Port GPIOE
#define Boton_S1_IT_EXTI_IRQn EXTI15_10_IRQn
#define Boton_S2_Pin GPIO_PIN_11
#define Boton_S2_GPIO_Port GPIOE
#define Boton_S3_Pin GPIO_PIN_12
#define Boton_S3_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOE
#define A_Pin GPIO_PIN_8
#define A_GPIO_Port GPIOD
#define B_Pin GPIO_PIN_9
#define B_GPIO_Port GPIOD
#define C_Pin GPIO_PIN_10
#define C_GPIO_Port GPIOD
#define D_Pin GPIO_PIN_11
#define D_GPIO_Port GPIOD
#define E_Pin GPIO_PIN_12
#define E_GPIO_Port GPIOD
#define F_Pin GPIO_PIN_13
#define F_GPIO_Port GPIOD
#define G_Pin GPIO_PIN_14
#define G_GPIO_Port GPIOD
#define TIM4_CH4_D15_Pin GPIO_PIN_15
#define TIM4_CH4_D15_GPIO_Port GPIOD
#define TIM3_CH1_C6_Pin GPIO_PIN_6
#define TIM3_CH1_C6_GPIO_Port GPIOC
#define TIM3_CH2_C7_Pin GPIO_PIN_7
#define TIM3_CH2_C7_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
