/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define PWR_BTN_Pin GPIO_PIN_14
#define PWR_BTN_GPIO_Port GPIOC
#define TPS_ENA_Pin GPIO_PIN_15
#define TPS_ENA_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOD
#define NTC_Pin GPIO_PIN_0
#define NTC_GPIO_Port GPIOA
#define VBAT_Pin GPIO_PIN_2
#define VBAT_GPIO_Port GPIOA
#define CURR_A_Pin GPIO_PIN_3
#define CURR_A_GPIO_Port GPIOA
#define CURR_B_Pin GPIO_PIN_4
#define CURR_B_GPIO_Port GPIOA
#define CURR_C_Pin GPIO_PIN_5
#define CURR_C_GPIO_Port GPIOA
#define VOLT_A_Pin GPIO_PIN_6
#define VOLT_A_GPIO_Port GPIOA
#define VOLT_B_Pin GPIO_PIN_7
#define VOLT_B_GPIO_Port GPIOA
#define HALL_C_Pin GPIO_PIN_0
#define HALL_C_GPIO_Port GPIOB
#define VOLT_C_Pin GPIO_PIN_1
#define VOLT_C_GPIO_Port GPIOB
#define PHA_A_L_Pin GPIO_PIN_13
#define PHA_A_L_GPIO_Port GPIOB
#define PHA_B_L_Pin GPIO_PIN_14
#define PHA_B_L_GPIO_Port GPIOB
#define PHA_C_L_Pin GPIO_PIN_15
#define PHA_C_L_GPIO_Port GPIOB
#define PHA_A_H_Pin GPIO_PIN_8
#define PHA_A_H_GPIO_Port GPIOA
#define PHA_B_H_Pin GPIO_PIN_9
#define PHA_B_H_GPIO_Port GPIOA
#define PHA_C_H_Pin GPIO_PIN_10
#define PHA_C_H_GPIO_Port GPIOA
#define LIGHT_Pin GPIO_PIN_15
#define LIGHT_GPIO_Port GPIOA
#define HALL_A_Pin GPIO_PIN_4
#define HALL_A_GPIO_Port GPIOB
#define HALL_B_Pin GPIO_PIN_5
#define HALL_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
