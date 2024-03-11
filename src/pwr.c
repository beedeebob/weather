/**
  ******************************************************************************
  * @file     	pwr.c
  * @author		ben
  * @version	1V0
  * @date		Mar 11, 2024
  * @brief
  */


/* Includes ------------------------------------------------------------------*/
#include "pwr.h"
#include "cmsis_os.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define PWR_TIMEOUT					1000
#define PWR_PUBLISHTIME				60000		//Every minute

#define PWR_TICKSINIT()				uint32_t startTicks = xTaskGetTickCount()
#define PWR_TICKSPASSED()			(xTaskGetTickCount() - startTicks)
#define PWR_TICKSLEFT(TO)			((PWR_TICKSPASSED > (TO)) ? 0 : ((TO) - PWR_TICKSPASSED))

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static osThreadId_t powerTaskHandle;
static const osThreadAttr_t powerTask_attributes =
{
  .name = "powerTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief	power thread initialize
  * @param	None
  * @retval	None
  */
void PWR_Init(void)
{
	//Configure Wake up timer for 1 minute

	powerTaskHandle = osThreadNew(PWR_Task, NULL, &powerTask_attributes);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	Task routine
  * @param	None
  * @retval	None
  */
void PWR_Task(void* arg)
{
	PWR_TICKSINIT();

	//Wait for weather readings
	osEventFlagsWait(systemFlags, SYS_EVENT_WEATHER | SYS_EVENT_WEATHERCMPLT, osFlagsNoClear | osFlagsWaitAny, PWR_TICKSLEFT(PWR_TIMEOUT));
	if(!(flags & SYS_EVENT_WEATHER))
	{
		//TODO: Shutdown
	}

	//Wait for esp readings
	osEventFlagsWait(systemFlags, SYS_EVENT_MQTTPUBLISHED | SYS_EVENT_ESPCMPLT, osFlagsNoClear | osFlagsWaitAny, PWR_TICKSLEFT(PWR_TIMEOUT));

	//Shutdown
	HAL_PWR_EnterSTOPMode(Regulator, STOPEntry);

	//Wake Up
	NVIC_SystemReset();
}
