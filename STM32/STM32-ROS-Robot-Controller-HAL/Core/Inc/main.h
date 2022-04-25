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
#include <stdio.h>
#include "cmsis_os.h"

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
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOC
#define OSCIN_Pin GPIO_PIN_0
#define OSCIN_GPIO_Port GPIOD
#define OSCOUT_Pin GPIO_PIN_1
#define OSCOUT_GPIO_Port GPIOD
#define M1IN1_Pin GPIO_PIN_0
#define M1IN1_GPIO_Port GPIOC
#define M1IN2_Pin GPIO_PIN_1
#define M1IN2_GPIO_Port GPIOC
#define M2IN1_Pin GPIO_PIN_2
#define M2IN1_GPIO_Port GPIOC
#define M2IN2_Pin GPIO_PIN_3
#define M2IN2_GPIO_Port GPIOC
#define M4EA_Pin GPIO_PIN_0
#define M4EA_GPIO_Port GPIOA
#define M4EB_Pin GPIO_PIN_1
#define M4EB_GPIO_Port GPIOA
#define USART2_TX_PI_Pin GPIO_PIN_2
#define USART2_TX_PI_GPIO_Port GPIOA
#define USART2_RX_PI_Pin GPIO_PIN_3
#define USART2_RX_PI_GPIO_Port GPIOA
#define M3IN1_Pin GPIO_PIN_4
#define M3IN1_GPIO_Port GPIOC
#define M3IN2_Pin GPIO_PIN_5
#define M3IN2_GPIO_Port GPIOC
#define VIN_Pin GPIO_PIN_0
#define VIN_GPIO_Port GPIOB
#define M4IN1_Pin GPIO_PIN_14
#define M4IN1_GPIO_Port GPIOB
#define M4IN2_Pin GPIO_PIN_15
#define M4IN2_GPIO_Port GPIOB
#define M1PWM_Pin GPIO_PIN_6
#define M1PWM_GPIO_Port GPIOC
#define M2PWM_Pin GPIO_PIN_7
#define M2PWM_GPIO_Port GPIOC
#define M3PWM_Pin GPIO_PIN_8
#define M3PWM_GPIO_Port GPIOC
#define M4PWM_Pin GPIO_PIN_9
#define M4PWM_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_8
#define KEY1_GPIO_Port GPIOA
#define USART1_TX_DEBUG_Pin GPIO_PIN_9
#define USART1_TX_DEBUG_GPIO_Port GPIOA
#define USART1_RX_DEBUG_Pin GPIO_PIN_10
#define USART1_RX_DEBUG_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define M1EA_Pin GPIO_PIN_15
#define M1EA_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_12
#define LED_BLUE_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_2
#define LED_GREEN_GPIO_Port GPIOD
#define M1EB_Pin GPIO_PIN_3
#define M1EB_GPIO_Port GPIOB
#define M2EA_Pin GPIO_PIN_4
#define M2EA_GPIO_Port GPIOB
#define M2EB_Pin GPIO_PIN_5
#define M2EB_GPIO_Port GPIOB
#define M3EA_Pin GPIO_PIN_6
#define M3EA_GPIO_Port GPIOB
#define M3EB_Pin GPIO_PIN_7
#define M3EB_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
