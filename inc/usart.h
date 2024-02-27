/*
 * usart.h
 *
 *  Created on: Feb 22, 2024
 *      Author: ben
 */

#ifndef USART_H_
#define USART_H_

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os2.h"
#include "stm32f3xx_hal.h"
#include "benQueue.h"

/* Exported defines ----------------------------------------------------------*/
#define USART_TXBUFFERSIZE				128
#define USART_RXBUFFERSIZE				128

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	UART_HandleTypeDef *huart;
	uint8_t txBuffer[USART_TXBUFFERSIZE];
	uint8_t rxBuffer[USART_RXBUFFERSIZE];
	uint32_t lastDMAIndex;
}USART_td;

/* Exported variables --------------------------------------------------------*/
extern const osThreadAttr_t usart_attributes;

extern USART_td usart1;

/* Exported functions ------------------------------------------------------- */
void USART_Init(void);
void USART_milli(void);
HAL_StatusTypeDef USART_Transmit(USART_td *usart, QUEUE_Typedef *pQueue);

#endif /* USART_H_ */
