/*
 * weather.c
 *
 *  Created on: Feb 17, 2024
 *      Author: ben
 */

/* Includes ------------------------------------------------------------------*/
#include "weather.h"
#include "cmsis_os2.h"
#include "bme280.h"

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const osThreadAttr_t weatherask_attributes =
{
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static BMP_TypeDef bme280;

/* Private function prototypes -----------------------------------------------*/
static BMP_ERR WTHR_TransmitReceive(struct BMP_TypeDef *arg, uint8_t *pTxData, uint8_t *pRxData, uint8_t length, uint32_t osTimeout);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief	Task routine
  * @param	None
  * @retval	None
  */
void WTHR_Task(const void* arg)
{
	//Initialize bme280
	BMP_InitTypeDef initStruct;
	BMP_StructInit(&initStruct);
	initStruct.transmitReceive = WTHR_TransmitReceive;

	BMP_init(&bme280, &initStruct);

	//TODO: Start BME280

	while(1)
	{
		//TODO: Get BME280 readings
		osDelay(1000);
	}
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	Transmit receive callback handler
  * @param	None
  * @retval	None
  */
static BMP_ERR WTHR_TransmitReceive(struct BMP_TypeDef *arg, uint8_t *pTxData, uint8_t *pRxData, uint8_t length, uint32_t osTimeout)
{
	if(!(pBMP->flags & BMP_FLAG_CMPLT))
		return BMP_ERRBUSY;

	//Clain the SPI
	if(pBMP->claimSPIInterface(pBMP) != BMP_ERROK)
		return BMP_ERRPERIPH;

	memcpy(pBMP->txData, pTx, length);

	//Transmit/Receive read of ID
	pBMP->chipSelect(pBMP);
	if(HAL_SPI_TransmitReceive_DMA(&hspi2, pTxData, pRxData, length) == HAL_OK)
		return BMP_ERROK;
	else
		return BMP_ERRBUSY;

	pBMP->chipUnSelect(pBMP);

	pBMP->flags &= ~BMP_FLAG_CMPLT;
	return BMP_ERROK;
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	SPI transmit/receive handler for the BME device
  * @param	arg: pointer to the BME device transmitting
  * @param	pTxData: pointer to the data to transmit
  * @param	pRxData: pointer to the receive buffer
  * @param	length: amount of data to transmit/receive
  * @retval	BMP_ERR
  */
static BMP_ERR WTHR_spiTransmitReceive(BMP_TypeDef *arg, uint8_t *pTxData, uint8_t *pRxData, uint8_t length)
{
	if(HAL_SPI_TransmitReceive_DMA(&hspi2, pTxData, pRxData, length) == HAL_OK)
		return BMP_ERROK;
	else
		return BMP_ERRBUSY;
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	SPI transmit/receive callback handler
  * @param	None
  * @retval	None
  */
void WTHR_spiTxRxCompleteCallback(void)
{
	BMP_transmitReceiveCompleteCallback(&bmp280);
}
