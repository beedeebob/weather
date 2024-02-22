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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_LED_Pin GPIO_PIN_13
#define GPIO_LED_GPIO_Port GPIOC
#define USART2_TX_PC_Pin GPIO_PIN_2
#define USART2_TX_PC_GPIO_Port GPIOA
#define USART2_RX_PC_Pin GPIO_PIN_3
#define USART2_RX_PC_GPIO_Port GPIOA
#define GPIO_BME280_CS_Pin GPIO_PIN_4
#define GPIO_BME280_CS_GPIO_Port GPIOA
#define SPI1_SCK_BME280_Pin GPIO_PIN_5
#define SPI1_SCK_BME280_GPIO_Port GPIOA
#define SPI1_MISO_BME280_Pin GPIO_PIN_6
#define SPI1_MISO_BME280_GPIO_Port GPIOA
#define SPI1_MOSI_BME280_Pin GPIO_PIN_7
#define SPI1_MOSI_BME280_GPIO_Port GPIOA
#define USART1_TX_ESP_Pin GPIO_PIN_9
#define USART1_TX_ESP_GPIO_Port GPIOA
#define USART1_RX_ESP_Pin GPIO_PIN_10
#define USART1_RX_ESP_GPIO_Port GPIOA
#define SWD_SWDIO_Pin GPIO_PIN_13
#define SWD_SWDIO_GPIO_Port GPIOA
#define SWD_SWCLK_Pin GPIO_PIN_14
#define SWD_SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
