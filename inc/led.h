/**
  ******************************************************************************
  * @file     	led.h
  * @author		ben
  * @version	1V0
  * @date		Feb 29, 2024
  * @brief		
  */

#ifndef LED_H_
#define LED_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void LED_Toggle(void);
void LED_ON(void);
void LED_OFF(void);
void LED_CMD(uint8_t state);

#endif /* LED_H_ */
