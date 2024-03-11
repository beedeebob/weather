/**
  ******************************************************************************
  * @file     	esp.c
  * @author		ben
  * @version	1V0
  * @date		Mar 5, 2024
  * @brief
  */


/* Includes ------------------------------------------------------------------*/
#include "esp.h"
#include "cmsis_os.h"
#include "main.h"
#include "comms.h"
#include "espPktIds.h"
#include "led.h"

/* Private define ------------------------------------------------------------*/
#define ESP_TIMEOUT					5000

#define ESP_TICKSINIT()				uint32_t startTicks = xTaskGetTickCount()
#define ESP_TICKSPASSED				(xTaskGetTickCount() - startTicks)
#define ESP_TICKSLEFT(TO)			((ESP_TICKSPASSED > (TO)) ? 0 : ((TO) - ESP_TICKSPASSED))
enum
{
	ESP_EVENT_WIFIACK = 0x01,
	ESP_EVENT_MQTTACK = 0x02,
	ESP_EVENT_WIFIOK = 0x04,
	ESP_EVENT_MQTTOK = 0x08,
	ESP_EVENT_WTHRACK = 0x10,
	ESP_EVENT_WTHROK= 0x20,
};
typedef enum
{
	ESP_ERR_OK = 0,
	ESP_ERR_TIMEOUT = -1
}ESP_ERR;

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static osThreadId_t espTaskHandle;
static const osThreadAttr_t espTask_attributes =
{
  .name = "espTask",
  .stack_size = 128,
  .priority = (osPriority_t) osPriorityNormal,
};
static uint8_t txBuffer[30];
static osEventFlagsId_t espEvents;
static const osEventFlagsAttr_t espEvent_attributes = {.name = "espEvents"};

/* Private function prototypes -----------------------------------------------*/
static void ESP_Startup(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief	ESP initialize
  * @param	None
  * @retval	None
  */
void ESP_Init(void)
{
	espTaskHandle = osThreadNew(ESP_task, NULL, &espTask_attributes);
	espEvents = osEventFlagsNew(&espEvent_attributes);
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	ESP control millisecond tick
  * @param	None
  * @retval	None
  */
void ESP_task(void *arg)
{
	ESP_TICKSINIT();

	//Hardware power up
	HAL_GPIO_WritePin(GPIO_ESP_NFLASH_GPIO_Port, GPIO_ESP_NFLASH_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIO_ESP_EN_GPIO_Port, GPIO_ESP_EN_Pin, GPIO_PIN_SET);

	osDelay(50);	//TODO: Verify time necessity

	//Start Wifi
	while(ESP_TICKSLEFT(ESP_TIMEOUT) > 0)
	{
		uint8_t len = 0;
		txBuffer[len++] = espPkt_WifiConnect;
		if(COMMS_ESPTransmit(txBuffer, len) == HAL_OK)
			osEventFlagsWait(espEvents, ESP_EVENT_WIFIACK, 0, 100);
		else
			osDelay(10);
	}

	//Start MQTT
	while(ESP_TICKSLEFT(ESP_TIMEOUT) > 0)
	{
		uint8_t len = 0;
		txBuffer[len++] = espPkt_StartMQTT;
		if(COMMS_ESPTransmit(txBuffer, len) == HAL_OK)
			osEventFlagsWait(espEvents, ESP_EVENT_MQTTOK, 0, 100);
		else
			osDelay(10);
	}

	//Wait for wifi and mqtt
	osEventFlagsWait(espEvents, (ESP_EVENT_WIFIOK | ESP_EVENT_MQTTOK), osFlagsWaitAll, ESP_TICKSLEFT(ESP_TIMEOUT));

	//Wait for weather update
	osEventFlagsWait(systemFlags, SYS_EVENT_WEATHER | SYS_EVENT_WEATHERCMPLT, osFlagsNoClear | osFlagsWaitAny, ESP_TICKSLEFT(ESP_TIMEOUT));
	if(!(flags & SYS_EVENT_WEATHER))
	{
		//Hardware shutdown
		HAL_GPIO_WritePin(GPIO_ESP_NFLASH_GPIO_Port, GPIO_ESP_NFLASH_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIO_ESP_EN_GPIO_Port, GPIO_ESP_EN_Pin, GPIO_PIN_RESET);

		osEventFlagsSet(systemEvent, SYS_EVENT_ESPCMPLT);
		osThreadTerminate(espTaskHandle);
		return;
	}

	//Transmit weather to ESP
	while(ESP_TICKSLEFT(ESP_TIMEOUT) > 0)
	{
		uint8_t len = 0;
		txBuffer[len++] = espPkt_SetWeather;
		txBuffer[len++] = (uint8_t)(temperature);
		txBuffer[len++] = (uint8_t)(temperature >> 8);
		txBuffer[len++] = (uint8_t)(temperature >> 16);
		txBuffer[len++] = (uint8_t)(temperature >> 24);
		txBuffer[len++] = (uint8_t)(pressure);
		txBuffer[len++] = (uint8_t)(pressure >> 8);
		txBuffer[len++] = (uint8_t)(pressure >> 16);
		txBuffer[len++] = (uint8_t)(pressure >> 24);
		txBuffer[len++] = (uint8_t)(humidity);
		txBuffer[len++] = (uint8_t)(humidity >> 8);
		txBuffer[len++] = (uint8_t)(humidity >> 16);
		txBuffer[len++] = (uint8_t)(humidity >> 24);
		if(COMMS_ESPTransmit(txBuffer, len) == HAL_OK)
			osEventFlagsWait(espEvents, ESP_EVENT_WTHRACK, 0, 100);
		else
			osDelay(10);
	}

	//Wait for publish
	osEventFlagsWait(espEvents, ESP_EVENT_WTHROK, 0, ESP_TICKSLEFT(ESP_TIMEOUT));

	//Hardware shutdown
	HAL_GPIO_WritePin(GPIO_ESP_NFLASH_GPIO_Port, GPIO_ESP_NFLASH_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIO_ESP_EN_GPIO_Port, GPIO_ESP_EN_Pin, GPIO_PIN_RESET);

	//Publish Complete
	if(flags & ESP_EVENT_WTHROK)
	{
		osEventFlagsSet(systemEvents, SYS_EVENT_MQTTPUBLISHED);

		//Flash LED
		LED_ON();
		osDelay(20);
		LED_OFF();
	}

	//Wait for weather update
	osEventFlagsSet(systemEvent, SYS_EVENT_ESPCMPLT);
	osThreadTerminate(espTaskHandle);
	return;
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	ESP state control system
  * @param	None
  * @retval	None
  */
void ESP_ESPCOMMSHandler(uint8_t *data, uint32_t length)
{
	switch(data[0])
	{
	case espPkt_ACK:
		if(data[1] == espPkt_WifiConnect)
			osEventFlagsSet(espEvents, ESP_EVENT_WIFIACK);
		else if(data[1] == espPkt_StartMQTT)
			osEventFlagsSet(espEvents, ESP_EVENT_MQTTACK);
		else if (data[1] == espPkt_SetWeather)
			osEventFlagsSet(espEvents, ESP_EVENT_WTHRACK);
		break;
	case espPkt_Status:
		if(data[1] & 0x01)
			osEventFlagsSet(espEvents, ESP_EVENT_WIFIOK);
		else
			osEventFlagsClear(espEvents, ESP_EVENT_WIFIOK);
		if(data[1] & 0x02)
			osEventFlagsSet(espEvents, ESP_EVENT_MQTTOK);
		else
			osEventFlagsClear(espEvents, ESP_EVENT_MQTTOK);
		if(data[1] & 0x04)
			osEventFlagsSet(espEvents, ESP_EVENT_WTHROK);
		break;
	}
}
