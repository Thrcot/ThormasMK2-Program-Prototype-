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
#include "stm32f3xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LS8_Pin GPIO_PIN_0
#define LS8_GPIO_Port GPIOA
#define LS9_Pin GPIO_PIN_1
#define LS9_GPIO_Port GPIOA
#define LS10_Pin GPIO_PIN_2
#define LS10_GPIO_Port GPIOA
#define LS11_Pin GPIO_PIN_3
#define LS11_GPIO_Port GPIOA
#define LS12_Pin GPIO_PIN_4
#define LS12_GPIO_Port GPIOA
#define LS13_Pin GPIO_PIN_5
#define LS13_GPIO_Port GPIOA
#define LS14_Pin GPIO_PIN_6
#define LS14_GPIO_Port GPIOA
#define LS15_Pin GPIO_PIN_7
#define LS15_GPIO_Port GPIOA
#define LS1_Pin GPIO_PIN_0
#define LS1_GPIO_Port GPIOB
#define LS2_Pin GPIO_PIN_1
#define LS2_GPIO_Port GPIOB
#define LS16_Pin GPIO_PIN_8
#define LS16_GPIO_Port GPIOA
#define LS17_Pin GPIO_PIN_11
#define LS17_GPIO_Port GPIOA
#define LS18_Pin GPIO_PIN_12
#define LS18_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_15
#define LD1_GPIO_Port GPIOA
#define LS3_Pin GPIO_PIN_3
#define LS3_GPIO_Port GPIOB
#define LS4_Pin GPIO_PIN_4
#define LS4_GPIO_Port GPIOB
#define LS5_Pin GPIO_PIN_5
#define LS5_GPIO_Port GPIOB
#define LS6_Pin GPIO_PIN_6
#define LS6_GPIO_Port GPIOB
#define LS7_Pin GPIO_PIN_7
#define LS7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
