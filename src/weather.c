/*
 * weather.c
 *
 *  Created on: Feb 17, 2024
 *      Author: ben
 */

/* Includes ------------------------------------------------------------------*/
#include "weather.h"
#include "bme280.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;


const osThreadAttr_t weatherTask_attributes =
{
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static osEventFlagsId_t weatherEventsHandle;
static const osEventFlagsAttr_t weatherEvent_attributes =
{
  .name = "weatherEvent"
};
#define weatherEventFlag_SPICOMPLETE			0x00000001

static BMP_TypeDef bme280;

/* Private function prototypes -----------------------------------------------*/
static BMP_ERR WTHR_TransmitReceive(struct BMP_TypeDef *arg, uint8_t *pTxData, uint8_t *pRxData, uint8_t length, uint32_t osTimeout);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief	Task routine
  * @param	None
  * @retval	None
  */
volatile int32_t temperature;
volatile uint32_t pressure;
volatile uint32_t humidity;
void WTHR_Task(void* arg)
{
	//Create event
	weatherEventsHandle = osEventFlagsNew(&weatherEvent_attributes);

	//Initialize bme280
	BMP_InitTypeDef initStruct;
	BMP_StructInit(&initStruct);
	initStruct.transmitReceive = WTHR_TransmitReceive;

	BMP_init(&bme280, &initStruct);

	//Start BME280
	BMP_Start(&bme280, osWaitForever);

	//Start LED
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	while(1)
	{
		//TODO: Get BME280 readings
		BMP_ReadSensors(&bme280, &temperature, &pressure, &humidity, osWaitForever);
		if(__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) == 0)
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
		else
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
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
	HAL_GPIO_WritePin(GPIO_CS_BME280_GPIO_Port, GPIO_CS_BME280_Pin, GPIO_PIN_RESET);

	osEventFlagsClear(weatherEventsHandle, weatherEventFlag_SPICOMPLETE);
	if(HAL_SPI_TransmitReceive_DMA(&hspi1, pTxData, pRxData, length) != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIO_CS_BME280_GPIO_Port, GPIO_CS_BME280_Pin, GPIO_PIN_SET);
		return BMP_ERRPERIPH;
	}

	uint32_t flags = osEventFlagsWait(weatherEventsHandle, weatherEventFlag_SPICOMPLETE, osFlagsWaitAll, osTimeout);
	if(!(flags & weatherEventFlag_SPICOMPLETE))
	{
		HAL_GPIO_WritePin(GPIO_CS_BME280_GPIO_Port, GPIO_CS_BME280_Pin, GPIO_PIN_SET);
		return BMP_ERRTIMEOUT;
	}

	HAL_GPIO_WritePin(GPIO_CS_BME280_GPIO_Port, GPIO_CS_BME280_Pin, GPIO_PIN_SET);
	return BMP_ERROK;
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	SPI transmit/receive callback handler
  * @param	None
  * @retval	None
  */
void WTHR_spiTxRxCompleteCallback(void)
{
	osEventFlagsSet(weatherEventsHandle, weatherEventFlag_SPICOMPLETE);
}
