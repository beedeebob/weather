/*
 * weather.h
 *
 *  Created on: Feb 17, 2024
 *      Author: ben
 */

#ifndef WEATHER_H_
#define WEATHER_H_

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os2.h"

/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern int32_t WTHR_Temperature;
extern uint32_t WTHR_Pressure;
extern uint32_t WTHR_Humidity;

/* Exported functions ------------------------------------------------------- */
void WTHR_Init(void);
void WTHR_spiTxRxCompleteCallback(void);

#endif /* WEATHER_H_ */
