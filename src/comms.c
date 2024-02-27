
/*
 * comms.c
 *
 *  Created on: Feb 22, 2024
 *      Author: ben
 */


/* Includes ------------------------------------------------------------------*/
#include "comms.h"
#include "benQueue.h"
#include "usart.h"

/* Private define ------------------------------------------------------------*/
#define COMMS_BUFFERSIZE				128

//Event FLAGS
enum
{
	COMMS_FLAG_TASKRUN = 0x01,
};

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t toUsart1Buffer[COMMS_BUFFERSIZE];
static QUEUE_Typedef toUSART1 = {toUsart1Buffer, COMMS_BUFFERSIZE, 0, 0};
static uint8_t toUsart2Buffer[COMMS_BUFFERSIZE];
static QUEUE_Typedef toUSART2 = {toUsart2Buffer, COMMS_BUFFERSIZE, 0, 0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief	Task milli routine
  * @param	None
  * @retval	None
  */
void COMMS_milli(void)
{
	if(QUEUE_COUNT(&toUSART1) > 0)
	{
		USART_Transmit(&usart1, &toUSART1);
	}

	if(QUEUE_COUNT(&toUSART2) > 0)
	{
		USART_Transmit(&usart2, &toUSART2);
	}
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	USB received callback handler
  * @param	None
  * @retval	None
  */
HAL_StatusTypeDef USART_ReceiveCallback(USART_td *usart, uint8_t *pData, uint8_t length)
{
	if(usart->huart->Instance == USART1)
	{
		if(QUEUE_AddArray(&toUSART2, pData, length) != QUEUE_OK)
			return HAL_ERROR;
	}
	if(usart->huart->Instance == USART2)
	{
		if(QUEUE_AddArray(&toUSART1, pData, length) != QUEUE_OK)
			return HAL_ERROR;
	}
	return HAL_OK;
}

