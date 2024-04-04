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
#include "stm32f2xx_hal.h"

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
#define LED_GN_Pin GPIO_PIN_13
#define LED_GN_GPIO_Port GPIOC
#define LED_YW_Pin GPIO_PIN_14
#define LED_YW_GPIO_Port GPIOC
#define LED_RD_Pin GPIO_PIN_15
#define LED_RD_GPIO_Port GPIOC
#define V_VEHI_Pin GPIO_PIN_0
#define V_VEHI_GPIO_Port GPIOC
#define V_ACCU_Pin GPIO_PIN_1
#define V_ACCU_GPIO_Port GPIOC
#define IMD_STATE_Pin GPIO_PIN_3
#define IMD_STATE_GPIO_Port GPIOA
#define WDI_Pin GPIO_PIN_4
#define WDI_GPIO_Port GPIOA
#define IMD_SAFE_Pin GPIO_PIN_5
#define IMD_SAFE_GPIO_Port GPIOA
#define SC_OPENING_Pin GPIO_PIN_6
#define SC_OPENING_GPIO_Port GPIOA
#define SC_OPEN_Pin GPIO_PIN_7
#define SC_OPEN_GPIO_Port GPIOA
#define SC_CLOSING_Pin GPIO_PIN_4
#define SC_CLOSING_GPIO_Port GPIOC
#define SC_STATE_Pin GPIO_PIN_5
#define SC_STATE_GPIO_Port GPIOC
#define TS_ACTIVATE_Pin GPIO_PIN_10
#define TS_ACTIVATE_GPIO_Port GPIOB
#define DANGER_V_Pin GPIO_PIN_11
#define DANGER_V_GPIO_Port GPIOB
#define AIR_P_ACT_Pin GPIO_PIN_14
#define AIR_P_ACT_GPIO_Port GPIOB
#define AIR_P_INT_Pin GPIO_PIN_15
#define AIR_P_INT_GPIO_Port GPIOB
#define AIR_N_ACT_Pin GPIO_PIN_6
#define AIR_N_ACT_GPIO_Port GPIOC
#define AIR_N_INT_Pin GPIO_PIN_7
#define AIR_N_INT_GPIO_Port GPIOC
#define AIR_P_SW_Pin GPIO_PIN_8
#define AIR_P_SW_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define PCHRG_ACT_Pin GPIO_PIN_2
#define PCHRG_ACT_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
