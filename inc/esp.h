/**
  ******************************************************************************
  * @file     	esp.h
  * @author		ben
  * @version	1V0
  * @date		Mar 5, 2024
  * @brief		
  */

#ifndef ESP_H_
#define ESP_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ESP_Init(void);
void ESP_ESPCOMMSHandler(uint8_t *data, uint32_t length);

#endif /* ESP_H_ */
