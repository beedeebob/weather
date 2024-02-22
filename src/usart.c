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

const osThreadAttr_t usart_attributes =
{
  .name = "usartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t usartTaskHandle1;
osThreadId_t usartTaskHandle2;

USART_td usart1 = {&huart1};
USART_td usart2 = {&huart2};

/* Private function prototypes -----------------------------------------------*/
HAL_StatusTypeDef USART_ReceiveCallback(USART_td *usart, uint8_t *pData, uint8_t length);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief	USART initialization
  * @param	None
  * @retval	None
  */
void USART_Init(void)
{
	usartTaskHandle1 = osThreadNew(USART_Task, &usart1, &usart_attributes);
	usartTaskHandle2 = osThreadNew(USART_Task, &usart2, &usart_attributes);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	Task routine
  * @param	None
  * @retval	None
  */
void USART_Task(void* arg)
{
	USART_td * usart = (USART_td*)arg;
	HAL_UART_Receive_DMA(usart->huart, usart->rxBuffer, USART_RXBUFFERSIZE);

	while(1)
	{
		uint32_t lastDMAIndex = 0 ;
		uint32_t currentDMAIndex = USART_RXBUFFERSIZE - __HAL_DMA_GET_COUNTER(usart->huart->hdmarx);
		uint32_t length = (currentDMAIndex - lastDMAIndex) & (USART_RXBUFFERSIZE - 1);
		if(length > 0)
		{
			if((lastDMAIndex + length) > USART_RXBUFFERSIZE)
				length = (USART_RXBUFFERSIZE - lastDMAIndex);

			if(USART_ReceiveCallback(usart, &usart->rxBuffer[lastDMAIndex], length) == HAL_OK)
				lastDMAIndex = ((lastDMAIndex + length) & (USART_RXBUFFERSIZE - 1));
		}

		osDelay(1);
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

	QUEUE_ReadArray(pQueue, usart->txBuffer, length);

	HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(usart->huart, usart->txBuffer, length);
	if(result == HAL_OK)
		QUEUE_Remove(pQueue, length);

	return result;
}
