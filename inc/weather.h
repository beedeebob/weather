/*
 * weather.h
 *
 *  Created on: Feb 17, 2024
 *      Author: ben
 */

#ifndef WEATHER_H_
#define WEATHER_H_

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern const osThreadAttr_t weatherTask_attributes;

/* Exported functions ------------------------------------------------------- */
void WTHR_Task(const void* arg);


#endif /* WEATHER_H_ */
