/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED_BRD_Pin GPIO_PIN_13
#define LED_BRD_GPIO_Port GPIOC
#define ADC_LDR_L1_Pin GPIO_PIN_0
#define ADC_LDR_L1_GPIO_Port GPIOA
#define ADC_LDR_L2_Pin GPIO_PIN_1
#define ADC_LDR_L2_GPIO_Port GPIOA
#define ADC_LDR_L3_Pin GPIO_PIN_2
#define ADC_LDR_L3_GPIO_Port GPIOA
#define ADC_LDR_R1_Pin GPIO_PIN_3
#define ADC_LDR_R1_GPIO_Port GPIOA
#define ADC_LDR_R2_Pin GPIO_PIN_4
#define ADC_LDR_R2_GPIO_Port GPIOA
#define ADC_LDR_R3_Pin GPIO_PIN_5
#define ADC_LDR_R3_GPIO_Port GPIOA
#define ADC_LDR_LF_Pin GPIO_PIN_6
#define ADC_LDR_LF_GPIO_Port GPIOA
#define ADC_LDR_RF_Pin GPIO_PIN_7
#define ADC_LDR_RF_GPIO_Port GPIOA
#define US_TRIG_Pin GPIO_PIN_0
#define US_TRIG_GPIO_Port GPIOB
#define US_ECHO_Pin GPIO_PIN_1
#define US_ECHO_GPIO_Port GPIOB
#define US_ECHO_EXTI_IRQn EXTI1_IRQn
#define LED_LDR_R3_Pin GPIO_PIN_12
#define LED_LDR_R3_GPIO_Port GPIOB
#define LED_LDR_R2_Pin GPIO_PIN_13
#define LED_LDR_R2_GPIO_Port GPIOB
#define LED_LDR_R1_Pin GPIO_PIN_14
#define LED_LDR_R1_GPIO_Port GPIOB
#define LED_LDR_L3_Pin GPIO_PIN_15
#define LED_LDR_L3_GPIO_Port GPIOB
#define LED_LDR_L2_Pin GPIO_PIN_8
#define LED_LDR_L2_GPIO_Port GPIOA
#define LED_LDR_L1_Pin GPIO_PIN_9
#define LED_LDR_L1_GPIO_Port GPIOA
#define PWM_SERVO_Pin GPIO_PIN_15
#define PWM_SERVO_GPIO_Port GPIOA
#define PWM_ESC_Pin GPIO_PIN_3
#define PWM_ESC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
