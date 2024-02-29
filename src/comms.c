
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
#include "usb_device.h"
#include "benPacket.h"

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
static uint8_t toUSBBuffer[COMMS_BUFFERSIZE];
static QUEUE_Typedef toUSB = {toUSBBuffer, COMMS_BUFFERSIZE, 0, 0};
static uint8_t bpktBuffer[COMMS_BUFFERSIZE];
static QUEUE_Typedef bpktQueue = {bpktBuffer, COMMS_BUFFERSIZE, 0, 0};

/* Private function prototypes -----------------------------------------------*/
static void COMMS_ESPPacketReceived(BPKT_Packet_TD *packet);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief	Task milli routine
  * @param	None
  * @retval	None
  */
void COMMS_milli(void)
{
	//Parse received data for packets
	BPKT_Packet_TD receivedPacket = {0};
	while(QUEUE_COUNT(&bpktQueue) > 0)
	{
		BPKT_STATUS_ENUM result = PKT_Decode(&bpktQueue, &receivedPacket);
		if(result == BPKT_OK)
		{
			COMMS_ESPPacketReceived(&receivedPacket);
			QUEUE_Remove(&bpktQueue, BPKT_PACKETSIZE(receivedPacket.length));
		}
		else if(result == BPKT_NOTENOUGHDATA)
			break;
		else
			QUEUE_Remove(&bpktQueue, 1);
	}

	//Transfer out to USART
	if(QUEUE_COUNT(&toUSART1) > 0)
	{
		USART_Transmit(&usart1, &toUSART1);
	}

	//Transfer out to USB
	if(QUEUE_COUNT(&toUSB) > 0)
	{
		USB_Transmit(&toUSB);
	}
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	USB received callback handler
  * @param	None
  * @retval	None
  */
HAL_StatusTypeDef USB_ReceiveCallback(uint8_t *pData, uint32_t length)
{
	if(QUEUE_AddArray(&toUSART1, pData, length) != QUEUE_OK)
		return HAL_ERROR;
	return HAL_OK;
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	USB received callback handler
  * @param	None
  * @retval	None
  */
HAL_StatusTypeDef USART_ReceiveCallback(USART_td *usart, uint8_t *pData, uint8_t length)
{
	QUEUE_AddArray(&toUSB, pData, length);
	if(QUEUE_AddArray(&bpktQueue, pData, length) != QUEUE_OK)
		return HAL_ERROR;
	return HAL_OK;
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	USB received callback handler
  * @param	None
  * @retval	None
  */
static void COMMS_ESPPacketReceived(BPKT_Packet_TD *packet)
{

}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	Transmit to ESP device
  * @param	data: pointer to the data to transit
  * @param	length: amount of data to transmit
  * @retval	HAL_StatusTypeDef
  */
HAL_StatusTypeDef COMMS_ESPTransmit(uint8_t *data, uint32_t length)
{
	if(PKT_Encode(data, length, &toUSART1) != BPKT_OK)
		return HAL_ERROR;
	return HAL_OK;
}

