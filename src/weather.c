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
#include "cmsis_os2.h"

/* Private define ------------------------------------------------------------*/
#define WTHR_TIMEOUT					1000

#define WTHR_TICKSINIT()				uint32_t startTicks = HAL_GetTick()
#define WTHR_TICKSPASSED()				(HAL_GetTick() - startTicks)
#define WTHR_TICKSLEFT(TO)				((WTHR_TICKSPASSED() > (TO)) ? 0 : ((TO) - WTHR_TICKSPASSED()))

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

static osThreadId_t weatherTaskHandle;
static const osThreadAttr_t weatherTask_attributes =
{
  .name = "defaultTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

static osEventFlagsId_t weatherEventsHandle;
static const osEventFlagsAttr_t weatherEvent_attributes =
{
  .name = "weatherEvent"
};
#define weatherEventFlag_SPICOMPLETE			0x00000001

static BMP_TypeDef bme280;

int32_t WTHR_Temperature;
uint32_t WTHR_Pressure;
uint32_t WTHR_Humidity;

/* Private function prototypes -----------------------------------------------*/
static BMP_ERR WTHR_TransmitReceive(struct BMP_TypeDef *arg, uint8_t *pTxData, uint8_t *pRxData, uint8_t length, uint32_t osTimeout);
static void WTHR_Task(void* arg);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief	Weather thread initialize
  * @param	None
  * @retval	None
  */
void WTHR_Init(void)
{
	weatherTaskHandle = osThreadNew(WTHR_Task, NULL, &weatherTask_attributes);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	Task routine
  * @param	None
  * @retval	None
  */
static void WTHR_Task(void* arg)
{
	WTHR_TICKSINIT();

	//Create event
	weatherEventsHandle = osEventFlagsNew(&weatherEvent_attributes);

	//Initialize bme280
	BMP_InitTypeDef initStruct;
	BMP_StructInit(&initStruct);
	initStruct.transmitReceive = WTHR_TransmitReceive;

	//Start the BME
	while(WTHR_TICKSLEFT(WTHR_TIMEOUT) > 0)
	{
		//Initialize the BME
		BMP_init(&bme280, &initStruct);

		//Start BME280
		if(BMP_Start(&bme280, WTHR_TICKSLEFT(WTHR_TIMEOUT)) != BMP_ERROK)
		{
			osDelay(100);
			continue;
		}
		break;
	}

	//Get weather readings
	while(WTHR_TICKSLEFT(WTHR_TIMEOUT) > 0)
	{
		int32_t temperature;
		uint32_t pressure;
		uint32_t humidity;
		if(BMP_ReadSensors(&bme280, &temperature, &pressure, &humidity, WTHR_TICKSLEFT(WTHR_TIMEOUT)) != BMP_ERROK)
		{
			osDelay(100);
			continue;
		}

		WTHR_Temperature = temperature;
		WTHR_Pressure = pressure;
		WTHR_Humidity = humidity;
		osEventFlagsSet(systemEvents, SYS_EVENT_WEATHER);
		break;
	}

	//Sleep BME device
	while(WTHR_TICKSLEFT(WTHR_TIMEOUT) > 0)
	{
		if(BMP_Sleep(&bme280, WTHR_TICKSLEFT(WTHR_TIMEOUT)) != BMP_ERROK)
		{
			osDelay(100);
			continue;
		}
		break;
	}

	osEventFlagsSet(systemEvents, SYS_EVENT_WEATHERCMPLT);
	osThreadTerminate(weatherTaskHandle);
	return;
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
