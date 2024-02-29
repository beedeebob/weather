/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v2.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
extern uint8_t UserTxBufferFS[64];

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* ---------------------------------------------------------------------------*/
/**
  * @brief	USB received callback handler
  * @param	None
  * @retval	None
  */
__weak HAL_StatusTypeDef USB_ReceiveCallback(uint8_t *pData, uint32_t length)
{
	return HAL_OK;
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief	USART transmit
  * @param	None
  * @retval	None
  */
HAL_StatusTypeDef USB_Transmit(QUEUE_Typedef *pQueue)
{
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	if (hcdc->TxState != 0)
		return HAL_BUSY;

	uint32_t length = QUEUE_COUNT(pQueue);
	if(length > 64)
		length = 64;

	QUEUE_ReadToArray(pQueue, 0, UserTxBufferFS, length);

	USBD_StatusTypeDef result = CDC_Transmit_FS(UserTxBufferFS, length);
	if(result == USBD_OK)
		QUEUE_Remove(pQueue, length);

	return result;
}

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */
	HAL_GPIO_WritePin(GPIO_USB_DPPU_GPIO_Port, GPIO_USB_DPPU_Pin, GPIO_PIN_SET);
  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

