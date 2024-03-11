/**
  ******************************************************************************
  * @file     	bmp280.h
  * @author		beede
  * @version	1V0
  * @date		Oct 22, 2023
  * @brief
  */


#ifndef BMP280_H_
#define BMP280_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported defines ----------------------------------------------------------*/
#define BMP_REG_CTRLHUM_OSRS_SKIPPED		0b000
#define BMP_REG_CTRLHUM_OSRS_OVRSMPX1		0b001
#define BMP_REG_CTRLHUM_OSRS_OVRSMPX2		0b010
#define BMP_REG_CTRLHUM_OSRS_OVRSMPX4		0b011
#define BMP_REG_CTRLHUM_OSRS_OVRSMPX8		0b100
#define BMP_REG_CTRLHUM_OSRS_OVRSMPX16		0b101

#define BMP_REG_STATUS_IMUPDATE				0x01	//Set when NVM data being transferred to image registers
#define BMP_REG_STATUS_MEASURING			0x01	//Set when conversion is running and cleared when results are available

#define BMP_REG_CTRLMEAS_MODE_SLEEP			0b00
#define BMP_REG_CTRLMEAS_MODE_FORCED		0b01
#define BMP_REG_CTRLMEAS_MODE_NORMAL		0b11

#define BMP_REG_CTRLMEAS_OSRSP_SKIPPED		0b000
#define BMP_REG_CTRLMEAS_OSRSP_OVRSMPX1		0b001
#define BMP_REG_CTRLMEAS_OSRSP_OVRSMPX2		0b010
#define BMP_REG_CTRLMEAS_OSRSP_OVRSMPX4		0b011
#define BMP_REG_CTRLMEAS_OSRSP_OVRSMPX8		0b100
#define BMP_REG_CTRLMEAS_OSRSP_OVRSMPX16	0b101

#define BMP_REG_CTRLMEAS_OSRST_SKIPPED		0b000
#define BMP_REG_CTRLMEAS_OSRST_OVRSMPX1		0b001
#define BMP_REG_CTRLMEAS_OSRST_OVRSMPX2		0b010
#define BMP_REG_CTRLMEAS_OSRST_OVRSMPX4		0b011
#define BMP_REG_CTRLMEAS_OSRST_OVRSMPX8		0b100
#define BMP_REG_CTRLMEAS_OSRST_OVRSMPX16	0b101

#define BMP_REG_CONFIG_SPI3WEN_EN			0x01	//ENABLE the SPI 3 wire interface

#define BMP_REG_CONFIG_FILTER_OFF			0b000	//IIR filter time constant
#define BMP_REG_CONFIG_FILTER_2				0b001
#define BMP_REG_CONFIG_FILTER_4				0b010
#define BMP_REG_CONFIG_FILTER_8				0b011
#define BMP_REG_CONFIG_FILTER_16			0b100

#define BMP_REG_CONFIG_TSB_0_5				0b000	//Standby inactive duration in milliseconds in normal mode
#define BMP_REG_CONFIG_TSB_62_5				0b001
#define BMP_REG_CONFIG_TSB_125				0b010
#define BMP_REG_CONFIG_TSB_250				0b011
#define BMP_REG_CONFIG_TSB_500				0b100
#define BMP_REG_CONFIG_TSB_1000				0b101
#define BMP_REG_CONFIG_TSB_10				0b110
#define BMP_REG_CONFIG_TSB_20				0b111

/* Exported types ------------------------------------------------------------*/

//ERRORS
typedef enum
{
	BMP_ERROK = 0,
	BMP_ERRBUSY = 1,
	BMP_ERRPERIPH = 2,
	BMP_ERRFULL = 3,
	BMP_ERRSTART = 4,
	BMP_ERRTIMEOUT= 5,
}BMP_ERR;

//TYPES
typedef struct
{
	uint16_t dig_t1;	/*! Calibration coefficients for the temperature sensor */
	int16_t dig_t2;
	int16_t dig_t3;

	uint16_t dig_p1;	/*! Calibration coefficients for the pressure sensor */
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	uint8_t dig_h1;
	int16_t dig_h2;
	uint8_t dig_h3;
	int16_t dig_h4;
	int16_t dig_h5;
	int8_t dig_h6;

	int32_t t_fine;		/*! Variable to store the intermediate temperature coefficient */
}BMP_Calib_TypeDef;
typedef struct
{
	uint8_t ctrl_hum;
	uint8_t status;
	uint8_t ctrl_meas;
	uint8_t config;
}BMP_Settings_TypeDef;
typedef struct BMP_TypeDef
{
	//Values
	uint8_t deviceID;
	BMP_Calib_TypeDef calibration;
	BMP_Settings_TypeDef settings;

	uint32_t pressure;
	int32_t temperature;
	uint32_t humidity;

	//Run time
	uint8_t flags;
	BMP_ERR (*transmitReceive)(struct BMP_TypeDef *arg, uint8_t *pTxData, uint8_t *pRxData, uint8_t length, uint32_t osTimeout);
}BMP_TypeDef;
typedef struct
{
	uint8_t mode;
	uint8_t pressureOversampling;
	uint8_t temperatureOversampling;
	uint8_t humidityOversampling;
	uint8_t spi3Wire;
	uint8_t iirFilter;
	uint8_t standbyInactiveDuration;

	BMP_ERR (*transmitReceive)(struct BMP_TypeDef *arg, uint8_t *pTxData, uint8_t *pRxData, uint8_t length, uint32_t osTimeout);
}BMP_InitTypeDef;

/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void BMP_StructInit(BMP_InitTypeDef *pInitStruct);
void BMP_init(BMP_TypeDef *pBME, BMP_InitTypeDef *pInitStruct);
BMP_ERR BMP_Start(BMP_TypeDef *pBMP, uint32_t osTimeout);

BMP_ERR BMP_ReadSensors(BMP_TypeDef *pBMP, int32_t *temperature, uint32_t *pressure, uint32_t *humidity, uint32_t osTimeout);

void BMP_milli(void);
BMP_ERR BMP_readSensors(BMP_TypeDef *pBME);
BMP_ERR BMP_Sleep(BMP_TypeDef *pBMP, uint32_t osTimeout);
BMP_ERR BMP_setPressureOversamling(BMP_TypeDef *pBME, uint8_t oversampling);
BMP_ERR BMP_setHumidityOversamling(BMP_TypeDef *pBME, uint8_t oversampling);
BMP_ERR BMP_setTemperatureOversamling(BMP_TypeDef *pBME, uint8_t oversampling);
BMP_ERR BMP_setSPI3Wire(BMP_TypeDef *pBME, uint8_t spi3Wire);
BMP_ERR BMP_setSPI3Wire(BMP_TypeDef *pBME, uint8_t spi3Wire);
BMP_ERR BMP_setIIRFilter(BMP_TypeDef *pBME, uint8_t iirFilter);
BMP_ERR BMP_setStandbyInactiveDuration(BMP_TypeDef *pBME, uint8_t duration);

void BMP_transmitReceiveCompleteCallback(BMP_TypeDef *arg);

#endif /* BMP280_H_ */
