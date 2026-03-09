/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

#include "stm32h5xx_nucleo.h"
#include <stdio.h>

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
#define OI_8_Pin GPIO_PIN_14
#define OI_8_GPIO_Port GPIOC
#define OI_9_Pin GPIO_PIN_15
#define OI_9_GPIO_Port GPIOC
#define OI_10_Pin GPIO_PIN_0
#define OI_10_GPIO_Port GPIOH
#define LO_10_Pin GPIO_PIN_1
#define LO_10_GPIO_Port GPIOH
#define LO_9_Pin GPIO_PIN_3
#define LO_9_GPIO_Port GPIOC
#define LO_6_Pin GPIO_PIN_6
#define LO_6_GPIO_Port GPIOA
#define LO_5_Pin GPIO_PIN_7
#define LO_5_GPIO_Port GPIOA
#define HO_2_Pin GPIO_PIN_4
#define HO_2_GPIO_Port GPIOC
#define OI_2_Pin GPIO_PIN_14
#define OI_2_GPIO_Port GPIOB
#define OI_1_Pin GPIO_PIN_15
#define OI_1_GPIO_Port GPIOB
#define LO_3_Pin GPIO_PIN_6
#define LO_3_GPIO_Port GPIOC
#define LO_2_Pin GPIO_PIN_7
#define LO_2_GPIO_Port GPIOC
#define OI_3_Pin GPIO_PIN_8
#define OI_3_GPIO_Port GPIOC
#define LO_4_Pin GPIO_PIN_9
#define LO_4_GPIO_Port GPIOC
#define LO_8_Pin GPIO_PIN_9
#define LO_8_GPIO_Port GPIOA
#define LO_7_Pin GPIO_PIN_10
#define LO_7_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define OI_5_Pin GPIO_PIN_10
#define OI_5_GPIO_Port GPIOC
#define OI_4_Pin GPIO_PIN_11
#define OI_4_GPIO_Port GPIOC
#define OI_7_Pin GPIO_PIN_12
#define OI_7_GPIO_Port GPIOC
#define OI_6_Pin GPIO_PIN_2
#define OI_6_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LO_4B4_Pin GPIO_PIN_4
#define LO_4B4_GPIO_Port GPIOB
#define HO_1_Pin GPIO_PIN_8
#define HO_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
