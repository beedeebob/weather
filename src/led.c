/**
  ******************************************************************************
  * @file     	led.c
  * @author		ben
  * @version	1V0
  * @date		Feb 29, 2024
  * @brief
  */


/* Includes ------------------------------------------------------------------*/
#include "led.h"
#include "stm32f3xx_hal.h"

/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

/* Private function prototypes -----------------------------------------------*/
static uint8_t LED_isON(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief	Toggle the LED
  * @param	None
  * @retval	None
  */
void LED_Toggle(void)
{
	//Control USB
	if(LED_isON())
		LED_OFF();
	else
		LED_ON();
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Turn the LED ON
  * @param	None
  * @retval	None
  */
void LED_ON(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Turn the LED OFF
  * @param	None
  * @retval	None
  */
void LED_OFF(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Change the state of the LED
  * @param	None
  * @retval	None
  */
void LED_CMD(uint8_t state)
{
	if(state)
		LED_ON();
	else
		LED_OFF();
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Get the LED state
  * @param	None
  * @retval	None
  */
static uint8_t LED_isON(void)
{
	if(__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) == 0)
		return 0;
	else
		return 1;
}
