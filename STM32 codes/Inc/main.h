/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define IR1_Pin GPIO_PIN_0
#define IR1_GPIO_Port GPIOC
#define IR1_EXTI_IRQn EXTI0_IRQn
#define IR2_Pin GPIO_PIN_1
#define IR2_GPIO_Port GPIOC
#define IR2_EXTI_IRQn EXTI1_IRQn
#define IR3_Pin GPIO_PIN_2
#define IR3_GPIO_Port GPIOC
#define IR3_EXTI_IRQn EXTI2_IRQn
#define IR4_Pin GPIO_PIN_3
#define IR4_GPIO_Port GPIOC
#define IR4_EXTI_IRQn EXTI3_IRQn
#define IR5_Pin GPIO_PIN_4
#define IR5_GPIO_Port GPIOC
#define IR5_EXTI_IRQn EXTI4_IRQn
#define TRIG_Pin GPIO_PIN_2
#define TRIG_GPIO_Port GPIOB
#define L_motor_Len_Pin GPIO_PIN_11
#define L_motor_Len_GPIO_Port GPIOB
#define L_motor_Ren_Pin GPIO_PIN_12
#define L_motor_Ren_GPIO_Port GPIOB
#define L_motor_Lis_Pin GPIO_PIN_13
#define L_motor_Lis_GPIO_Port GPIOB
#define L_motor_Ris_Pin GPIO_PIN_14
#define L_motor_Ris_GPIO_Port GPIOB
#define R_motor_Ren_Pin GPIO_PIN_12
#define R_motor_Ren_GPIO_Port GPIOD
#define R_motor_Ris_Pin GPIO_PIN_13
#define R_motor_Ris_GPIO_Port GPIOD
#define R_motor_Len_Pin GPIO_PIN_14
#define R_motor_Len_GPIO_Port GPIOD
#define R_motor_Lis_Pin GPIO_PIN_15
#define R_motor_Lis_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
