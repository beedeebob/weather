/*
 * usart.c
 *
 *  Created on: Feb 22, 2024
 *      Author: ben
 */


/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* Private define ------------------------------------------------------------*/

//Event FLAGS
enum
{
	USART_FLAG_TASKRUN = 0x01,
};

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

USART_td usartESP = {&huart1};
USART_td usartPC = {&huart2};

/* Private function prototypes -----------------------------------------------*/
static void USART_usartTick(USART_td *usart);
HAL_StatusTypeDef USART_ReceiveCallback(USART_td *usart, uint8_t *pData, uint8_t length);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief	USART initialization
  * @param	None
  * @retval	None
  */
void USART_Init(void)
{
	HAL_UART_Receive_DMA(usartESP.huart, usartESP.rxBuffer, USART_RXBUFFERSIZE);
	HAL_UART_Receive_DMA(usartPC.huart, usartPC.rxBuffer, USART_RXBUFFERSIZE);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	Tick routine
  * @param	None
  * @retval	None
  */
void USART_milli(void)
{
	USART_usartTick(&usartESP);
	USART_usartTick(&usartPC);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	Tick routine
  * @param	None
  * @retval	None
  */
static void USART_usartTick(USART_td *usart)
{
	uint32_t currentDMAIndex = USART_RXBUFFERSIZE - __HAL_DMA_GET_COUNTER(usart->huart->hdmarx);
	uint32_t length = (currentDMAIndex - usart->lastDMAIndex) & (USART_RXBUFFERSIZE - 1);
	if(length > 0)
	{
		if((usart->lastDMAIndex + length) > USART_RXBUFFERSIZE)
			length = (USART_RXBUFFERSIZE - usart->lastDMAIndex);

		if(USART_ReceiveCallback(usart, &usart->rxBuffer[usart->lastDMAIndex], length) == HAL_OK)
			usart->lastDMAIndex = ((usart->lastDMAIndex + length) & (USART_RXBUFFERSIZE - 1));
	}
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	USB received callback handler
  * @param	None
  * @retval	None
  */
__weak HAL_StatusTypeDef USART_ReceiveCallback(USART_td *usart, uint8_t *pData, uint8_t length)
{
	return HAL_OK;
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	USART transmit
  * @param	None
  * @retval	None
  */
HAL_StatusTypeDef USART_Transmit(USART_td *usart, QUEUE_Typedef *pQueue)
{
	if(usart->huart->gState != HAL_UART_STATE_READY)
		return HAL_BUSY;

	uint32_t length = QUEUE_COUNT(pQueue);
	if(length > USART_TXBUFFERSIZE)
		length = USART_TXBUFFERSIZE;

	QUEUE_ReadToArray(pQueue, 0, usart->txBuffer, length);

	HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(usart->huart, usart->txBuffer, length);
	if(result == HAL_OK)
		QUEUE_Remove(pQueue, length);

	return result;
}
