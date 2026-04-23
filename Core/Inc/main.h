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
#define OI_10_EXTI_IRQn EXTI0_IRQn
#define LMO_6_Pin GPIO_PIN_1
#define LMO_6_GPIO_Port GPIOH
#define LMO_5_Pin GPIO_PIN_3
#define LMO_5_GPIO_Port GPIOC
#define LMO_2_Pin GPIO_PIN_6
#define LMO_2_GPIO_Port GPIOA
#define LMO_1_Pin GPIO_PIN_7
#define LMO_1_GPIO_Port GPIOA
#define HMO_2_Pin GPIO_PIN_4
#define HMO_2_GPIO_Port GPIOC
#define OI_2_Pin GPIO_PIN_14
#define OI_2_GPIO_Port GPIOB
#define OI_2_EXTI_IRQn EXTI14_IRQn
#define OI_1_Pin GPIO_PIN_15
#define OI_1_GPIO_Port GPIOB
#define OI_1_EXTI_IRQn EXTI15_IRQn
#define LIO_3_Pin GPIO_PIN_6
#define LIO_3_GPIO_Port GPIOC
#define LIO_2_Pin GPIO_PIN_7
#define LIO_2_GPIO_Port GPIOC
#define OI_3_Pin GPIO_PIN_8
#define OI_3_GPIO_Port GPIOC
#define OI_3_EXTI_IRQn EXTI8_IRQn
#define LIO_4_Pin GPIO_PIN_9
#define LIO_4_GPIO_Port GPIOC
#define LMO_4_Pin GPIO_PIN_9
#define LMO_4_GPIO_Port GPIOA
#define LMO_3_Pin GPIO_PIN_10
#define LMO_3_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define OI_5_Pin GPIO_PIN_10
#define OI_5_GPIO_Port GPIOC
#define OI_5_EXTI_IRQn EXTI10_IRQn
#define OI_4_Pin GPIO_PIN_11
#define OI_4_GPIO_Port GPIOC
#define OI_4_EXTI_IRQn EXTI11_IRQn
#define OI_7_Pin GPIO_PIN_12
#define OI_7_GPIO_Port GPIOC
#define OI_7_EXTI_IRQn EXTI12_IRQn
#define OI_6_Pin GPIO_PIN_2
#define OI_6_GPIO_Port GPIOD
#define OI_6_EXTI_IRQn EXTI2_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LIO_1_Pin GPIO_PIN_4
#define LIO_1_GPIO_Port GPIOB
#define HMO_1_Pin GPIO_PIN_8
#define HMO_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
