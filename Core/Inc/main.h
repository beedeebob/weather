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
#include "cmsis_os.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern osEventFlagsId_t systemEvents;

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
#define TIM1_LED_Pin GPIO_PIN_13
#define TIM1_LED_GPIO_Port GPIOC
#define GPIO_CS_BME280_Pin GPIO_PIN_4
#define GPIO_CS_BME280_GPIO_Port GPIOA
#define SPI1_SCK_BME280_Pin GPIO_PIN_5
#define SPI1_SCK_BME280_GPIO_Port GPIOA
#define SPI1_MISO_BME280_Pin GPIO_PIN_6
#define SPI1_MISO_BME280_GPIO_Port GPIOA
#define SPI1_MOSI_BME280_Pin GPIO_PIN_7
#define SPI1_MOSI_BME280_GPIO_Port GPIOA
#define GPIO_ESP_EN_Pin GPIO_PIN_8
#define GPIO_ESP_EN_GPIO_Port GPIOA
#define USART1_TX_ESP8266_Pin GPIO_PIN_9
#define USART1_TX_ESP8266_GPIO_Port GPIOA
#define USART1_RX_ESP8266_Pin GPIO_PIN_10
#define USART1_RX_ESP8266_GPIO_Port GPIOA
#define GPIO_ESP_NFLASH_Pin GPIO_PIN_11
#define GPIO_ESP_NFLASH_GPIO_Port GPIOA
#define GPIO_USB_DPPU_Pin GPIO_PIN_15
#define GPIO_USB_DPPU_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

enum
{
	SYS_EVENT_WEATHER = 0x01,		//Weather readings available
	SYS_EVENT_MQTTPUBLISHED = 0x02,	//Weather published over MQTT
	SYS_EVENT_WEATHERCMPLT = 0x04,	//Weather done processing
	SYS_EVENT_ESPCMPLT = 0x08,		//ESP done processing
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
