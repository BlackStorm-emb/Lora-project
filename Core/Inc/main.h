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
#define but_1_Pin GPIO_PIN_13
#define but_1_GPIO_Port GPIOC
#define but_2_Pin GPIO_PIN_14
#define but_2_GPIO_Port GPIOC
#define TEMP_Pin GPIO_PIN_0
#define TEMP_GPIO_Port GPIOA
#define Cell_Voltage_Pin GPIO_PIN_1
#define Cell_Voltage_GPIO_Port GPIOA
#define LORA_RST_Pin GPIO_PIN_2
#define LORA_RST_GPIO_Port GPIOA
#define LORA_DIO0_Pin GPIO_PIN_3
#define LORA_DIO0_GPIO_Port GPIOA
#define LORA_NSS_Pin GPIO_PIN_4
#define LORA_NSS_GPIO_Port GPIOA
#define Relay_Pin GPIO_PIN_0
#define Relay_GPIO_Port GPIOB
#define HL4_Pin GPIO_PIN_1
#define HL4_GPIO_Port GPIOB
#define HL3_Pin GPIO_PIN_2
#define HL3_GPIO_Port GPIOB
#define BL_E_Pin GPIO_PIN_12
#define BL_E_GPIO_Port GPIOB
#define LCD_DB7_Pin GPIO_PIN_13
#define LCD_DB7_GPIO_Port GPIOB
#define LCD_DB6_Pin GPIO_PIN_14
#define LCD_DB6_GPIO_Port GPIOB
#define LCD_DB5_Pin GPIO_PIN_15
#define LCD_DB5_GPIO_Port GPIOB
#define LCD_DB4_Pin GPIO_PIN_8
#define LCD_DB4_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_9
#define LCD_E_GPIO_Port GPIOA
#define LCD_A0_Pin GPIO_PIN_10
#define LCD_A0_GPIO_Port GPIOA
#define PA12_Pin GPIO_PIN_4
#define PA12_GPIO_Port GPIOB
#define PA11_Pin GPIO_PIN_5
#define PA11_GPIO_Port GPIOB
#define PB5_Pin GPIO_PIN_6
#define PB5_GPIO_Port GPIOB
#define PB8_Pin GPIO_PIN_7
#define PB8_GPIO_Port GPIOB
#define PB7_Pin GPIO_PIN_8
#define PB7_GPIO_Port GPIOB
#define PB6_Pin GPIO_PIN_9
#define PB6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
