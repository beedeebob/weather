/*
 * comms.h
 *
 *  Created on: Feb 22, 2024
 *      Author: ben
 */

#ifndef COMMS_H_
#define COMMS_H_

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os2.h"
#include "stm32f3xx_hal.h"

/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern const osThreadAttr_t comms_attributes;

/* Exported functions ------------------------------------------------------- */
void COMMS_milli(void);
HAL_StatusTypeDef COMMS_ESPTransmit(uint8_t *data, uint32_t length);

#endif /* COMMS_H_ */
