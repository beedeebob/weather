/**
  ******************************************************************************
  * @file     	bmp280.c
  * @author		beede
  * @version	1V0
  * @date		Oct 14, 2023
  * @brief
  */

/* Includes ------------------------------------------------------------------*/
#include <bme280.h>
#include "stdint.h"
#include "main.h"
#include "string.h"

/* Private define ------------------------------------------------------------*/
//MACROS
#define TOU16(ARRAY, INDEX)				(ARRAY[INDEX] + (ARRAY[INDEX + 1] << 8))

//REGISTERS
#define BMP_REG_CALIB00					0x88	//BMP_REG_calib00 - BMP_REG_calib25 -> 0x88 - 0xA1
#define BMP_REG_CALIB25					0xA1	//
#define BMP_REG_ID						0xD0	//
#define BMP_REG_RESET					0xE0	//
#define BMP_REG_RESET_RST				0xB6
#define BMP_REG_CALIB26					0xE1	//BMP_REG_calib00 - BMP_REG_calib25 -> 0x88 - 0xA1
#define BMP_REG_CALIB41					0xF0	//
#define BMP_REG_CTRLHUM					0xF2	//
#define BMP_REG_CTRLHUM_OSRSH_POS		0x00
#define BMP_REG_CTRLHUM_OSRSH_BITS		0x07
#define BMP_REG_CTRLHUM_OSRSH_MASK		(BMP_REG_CTRLHUM_OSRSH_BITS << BMP_REG_CTRLHUM_OSRSH_POS)
#define BMP_REG_STATUS					0xF3
#define BMP_REG_STATUS_IMUPDATE_POS		0x00
#define BMP_REG_STATUS_IMUPDATE_BITS	0x01
#define BMP_REG_STATUS_IMUPDATE_MASK	(BMP_REG_STATUS_IMUPDATE_BITS << BMP_REG_STATUS_IMUPDATE_POS)
#define BMP_REG_STATUS_MEASURING_POS	0x03
#define BMP_REG_STATUS_MEASURING_BITS	0x01
#define BMP_REG_STATUS_MEASURING_MASK	(BMP_REG_STATUS_MEASURING_BITS << BMP_REG_STATUS_MEASURING_POS)
#define BMP_REG_CTRLMEAS				0xF4
#define BMP_REG_CTRLMEAS_MODE_POS		0x00
#define BMP_REG_CTRLMEAS_MODE_BITS		0x03
#define BMP_REG_CTRLMEAS_MODE_MASK		(BMP_REG_CTRLMEAS_MODE_BITS << BMP_REG_CTRLMEAS_MODE_POS)
#define BMP_REG_CTRLMEAS_OSRSP_POS		0x02
#define BMP_REG_CTRLMEAS_OSRSP_BITS		0x07
#define BMP_REG_CTRLMEAS_OSRSP_MASK		(BMP_REG_CTRLMEAS_OSRSP_BITS << BMP_REG_CTRLMEAS_OSRSP_POS)
#define BMP_REG_CTRLMEAS_OSRST_POS		0x05
#define BMP_REG_CTRLMEAS_OSRST_BITS		0x07
#define BMP_REG_CTRLMEAS_OSRST_MASK		(BMP_REG_CTRLMEAS_OSRST_BITS << BMP_REG_CTRLMEAS_OSRST_POS)
#define BMP_REG_CONFIG					0xF5
#define BMP_REG_CONFIG_SPI3WEN_POS		0x00
#define BMP_REG_CONFIG_SPI3WEN_BITS		0x01
#define BMP_REG_CONFIG_SPI3WEN_MASK		(BMP_REG_CONFIG_SPI3WEN_BITS << BMP_REG_CONFIG_SPI3WEN_POS)
#define BMP_REG_CONFIG_FILTER_POS		0x02
#define BMP_REG_CONFIG_FILTER_BITS		0x07
#define BMP_REG_CONFIG_FILTER_MASK		(BMP_REG_CONFIG_FILTER_BITS << BMP_REG_CONFIG_FILTER_POS)
#define BMP_REG_CONFIG_TSB_POS			0x05
#define BMP_REG_CONFIG_TSB_BITS			0x07
#define BMP_REG_CONFIG_TSB_MASK			(BMP_REG_CONFIG_TSB_BITS << BMP_REG_CONFIG_TSB_POS)
#define BMP_REG_PRESS_MSB				0xF7
#define BMP_REG_PRESS_LSB				0xF8
#define BMP_REG_PRESS_XLSB				0xF9
#define BMP_REG_TEMP_MSB				0xFA
#define BMP_REG_TEMP_LSB				0xFB
#define BMP_REG_TEMP_XLSB				0xFC
#define BMP_REG_HUM_MSB					0xFD
#define BMP_REG_HUM_LSB					0xFE

//FLAGS
enum BMP_FLAGS
{
	BMP_FLAG_STARTED = 0x01,
};

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int32_t compensate_temperature(BMP_TypeDef *pBMP, int32_t rawTemperature);
static uint32_t compensate_pressure(BMP_TypeDef *pBMP, int32_t rawPressure);
static uint32_t compensate_humidity(BMP_TypeDef *pBMP, int32_t rawHumidity);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief	Initialization of a BME IC struct
  * @param	None
  * @retval	None
  */
void BMP_init(BMP_TypeDef *pBMP, BMP_InitTypeDef *pInitStruct)
{
	//Initialize device
	memset(pBMP, 0, sizeof(BMP_TypeDef));
	pBMP->transmitReceive = pInitStruct->transmitReceive;

	pBMP->settings.ctrl_hum &= ~(BMP_REG_CTRLHUM_OSRSH_MASK);
	pBMP->settings.ctrl_hum |= (BMP_REG_CTRLHUM_OSRSH_MASK & (pInitStruct->humidityOversampling << BMP_REG_CTRLHUM_OSRSH_POS));

	pBMP->settings.ctrl_meas &= ~(BMP_REG_CTRLMEAS_MODE_MASK | BMP_REG_CTRLMEAS_OSRST_MASK | BMP_REG_CTRLMEAS_OSRSP_MASK);
	pBMP->settings.ctrl_meas |= (BMP_REG_CTRLMEAS_MODE_MASK & (pInitStruct->mode << BMP_REG_CTRLMEAS_MODE_POS));
	pBMP->settings.ctrl_meas |= (BMP_REG_CTRLMEAS_OSRST_MASK & (pInitStruct->temperatureOversampling << BMP_REG_CTRLMEAS_OSRST_POS));
	pBMP->settings.ctrl_meas |= (BMP_REG_CTRLMEAS_OSRSP_MASK & (pInitStruct->pressureOversampling << BMP_REG_CTRLMEAS_OSRSP_POS));

	pBMP->settings.config &= ~(BMP_REG_CONFIG_SPI3WEN_MASK |BMP_REG_CONFIG_FILTER_MASK |BMP_REG_CONFIG_TSB_MASK);
	pBMP->settings.config |= (BMP_REG_CONFIG_SPI3WEN_MASK & (pInitStruct->spi3Wire << BMP_REG_CONFIG_SPI3WEN_POS));
	pBMP->settings.config |= (BMP_REG_CONFIG_FILTER_MASK & (pInitStruct->iirFilter << BMP_REG_CONFIG_FILTER_POS));
	pBMP->settings.config |= (BMP_REG_CONFIG_TSB_MASK & (pInitStruct->standbyInactiveDuration << BMP_REG_CONFIG_TSB_POS));
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Initialization of a BME initialization struct to a default system
  * @param	None
  * @retval	None
  */
void BMP_StructInit(BMP_InitTypeDef *pInitStruct)
{
	memset(pInitStruct, 0, sizeof(BMP_InitTypeDef));
	pInitStruct->mode = BMP_REG_CTRLMEAS_MODE_NORMAL;
	pInitStruct->temperatureOversampling = BMP_REG_CTRLMEAS_OSRST_OVRSMPX1;
	pInitStruct->pressureOversampling = BMP_REG_CTRLMEAS_OSRSP_OVRSMPX1;
	pInitStruct->humidityOversampling = BMP_REG_CTRLHUM_OSRS_OVRSMPX1;
	pInitStruct->iirFilter = BMP_REG_CONFIG_FILTER_OFF;
	pInitStruct->spi3Wire = 0;

	pInitStruct->transmitReceive = NULL;
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Run the startup procedure of the BME280 device
  * @param	pBMP: pointer to the BME struct
  * @param	osTimeout: timeout for the os
  * @retval	BMP_ERR
  */
BMP_ERR BMP_Start(BMP_TypeDef *pBMP, uint32_t osTimeout)
{
	uint8_t data[(BMP_REG_CALIB25 - BMP_REG_CALIB00 + 2)];	//Worst case

	//Get ID
	data[0] = BMP_REG_ID | 0x80;
	data[1] = 0;
	BMP_ERR result = pBMP->transmitReceive(pBMP, data, data, 2, osTimeout);
	if(result != BMP_ERROK)
		return result;

	pBMP->deviceID = data[1];

	//Reset
	data[0] = (BMP_REG_RESET & ~0x80);
	data[1] = BMP_REG_RESET_RST;
	result = pBMP->transmitReceive(pBMP, data, data, 2, osTimeout);
	if(result != BMP_ERROK)
		return result;

	//Calibration 0 - 25
	data[0] = (BMP_REG_CALIB00 | 0x80);
	result = pBMP->transmitReceive(pBMP, data, data, (BMP_REG_CALIB25 - BMP_REG_CALIB00 + 2), osTimeout);
	if(result != BMP_ERROK)
		return result;

	pBMP->calibration.dig_t1 = TOU16(data, 1);
    pBMP->calibration.dig_t2 = (int16_t)TOU16(data, 3);
    pBMP->calibration.dig_t3 = (int16_t)TOU16(data, 5);
    pBMP->calibration.dig_p1 = TOU16(data, 7);
    pBMP->calibration.dig_p2 = (int16_t)TOU16(data, 9);
    pBMP->calibration.dig_p3 = (int16_t)TOU16(data, 11);
    pBMP->calibration.dig_p4 = (int16_t)TOU16(data, 13);
    pBMP->calibration.dig_p5 = (int16_t)TOU16(data, 15);
    pBMP->calibration.dig_p6 = (int16_t)TOU16(data, 17);
    pBMP->calibration.dig_p7 = (int16_t)TOU16(data, 19);
    pBMP->calibration.dig_p8 = (int16_t)TOU16(data, 21);
    pBMP->calibration.dig_p9 = (int16_t)TOU16(data, 23);
    pBMP->calibration.dig_h1 = (uint8_t)data[26];

	//Calibration 26 - 41
	data[0] = (BMP_REG_CALIB26 | 0x80);
	result = pBMP->transmitReceive(pBMP, data, data, (BMP_REG_CALIB41 - BMP_REG_CALIB26 + 2), osTimeout);
	if(result != BMP_ERROK)
		return result;

    pBMP->calibration.dig_h2 = (int16_t)TOU16(data, 1);
    pBMP->calibration.dig_h3 = (uint8_t)data[3];
    pBMP->calibration.dig_h4 = (int16_t)TOU16(data, 4);
    pBMP->calibration.dig_h5 = (int16_t)TOU16(data, 6);
    pBMP->calibration.dig_h6 = (int16_t)TOU16(data, 8);

    //Get Settings
	data[0] = (BMP_REG_CTRLHUM | 0x80);
	result = pBMP->transmitReceive(pBMP, data, data, (BMP_REG_CONFIG - BMP_REG_CTRLHUM + 2), osTimeout);
	if(result != BMP_ERROK)
		return result;

	//Update Settings
	if(pBMP->settings.ctrl_hum != data[1])
	{
		data[0] = ((BMP_REG_CTRLHUM) & ~0x80);
		data[1] = pBMP->settings.ctrl_hum;
		result = pBMP->transmitReceive(pBMP, data, data, 2, osTimeout);
		if(result != BMP_ERROK)
			return result;
	}
	if(pBMP->settings.ctrl_meas != data[3])
	{
		uint8_t data[2];
		data[0] = ((BMP_REG_CTRLMEAS) & ~0x80);
		data[1] = pBMP->settings.ctrl_meas;
		result = pBMP->transmitReceive(pBMP, data, data, 2, osTimeout);
		if(result != BMP_ERROK)
			return result;
	}
	if(pBMP->settings.config != data[4])
	{
		uint8_t data[2];
		data[0] = ((BMP_REG_CONFIG) & ~0x80);
		data[1] = pBMP->settings.config;
		result = pBMP->transmitReceive(pBMP, data, data, 2, osTimeout);
		if(result != BMP_ERROK)
			return result;
	}

	pBMP->flags |= BMP_FLAG_STARTED;
	return BMP_ERROK;
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Read all sensors
  * @param	pBMP: pointer to the BME struct
  * @param[out]	temperature: pointer to the returned temperature in 0.01 degrees C
  * @param[out]	pressure: pointer to the returned pressure in Q24.8 (24 integer bits and 8 fractional bits)(Divide by 256 for Pa)
  * @param[out]	humidity: pointer to the returned humidity in Q22.10 (22 integer bits and 10 fractional bits)(Divide by 1024 for %RH)
  * @param	osTimeout: os timeout
  * @retval	BMP_ERR
  */
BMP_ERR BMP_ReadSensors(BMP_TypeDef *pBMP, int32_t *temperature, uint32_t *pressure, uint32_t *humidity, uint32_t osTimeout)
{
	if(!(pBMP->flags & BMP_FLAG_STARTED))
		return BMP_ERRSTART;

	//Read the sensor data
	uint8_t data[(BMP_REG_HUM_LSB - BMP_REG_PRESS_MSB + 2)];
	data[0] = (BMP_REG_PRESS_MSB | 0x80);
	BMP_ERR result = pBMP->transmitReceive(pBMP, data, data, (BMP_REG_HUM_LSB - BMP_REG_PRESS_MSB + 2), osTimeout);
	if(result != BMP_ERROK)
		return result;

	//Calculate the result
	int32_t rawPressure = (data[1] << 12) + (data[2] << 4) + (data[3] >> 4);
	int32_t rawTemperature = (data[4] << 12) + (data[5] << 4) + (data[6] >> 4);
	int32_t rawHumidity = (data[7] << 8) + data[8];

	pBMP->temperature = compensate_temperature(pBMP, rawTemperature);
	pBMP->pressure = compensate_pressure(pBMP, rawPressure);
	pBMP->humidity = compensate_humidity(pBMP, rawHumidity);

    *temperature = pBMP->temperature;
    *pressure = pBMP->pressure;
    *humidity = pBMP->humidity;
	return BMP_ERROK;
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Compensate temperature
  * @param	pBMP: pointer to the BME struct
  * @retval	None
  */
static int32_t compensate_temperature(BMP_TypeDef *pBMP, int32_t rawTemperature)
{
	int32_t var1, var2, t;
	var1 = ((((rawTemperature >> 3) - ((int32_t)pBMP->calibration.dig_t1 << 1))) * ((int32_t)pBMP->calibration.dig_t2)) >> 11;
	var2 = (((((rawTemperature >> 4) - ((int32_t)pBMP->calibration.dig_t1)) * ((rawTemperature >> 4) - ((int32_t)pBMP->calibration.dig_t1))) >> 12) * ((int32_t)pBMP->calibration.dig_t3)) >> 14;
	pBMP->calibration.t_fine = var1 + var2;
	t = (pBMP->calibration.t_fine * 5 + 128) >> 8;
	return t;
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Compensate pressure
  * @param	pBMP: pointer to the BME struct
  * @retval	None
  */
static uint32_t compensate_pressure(BMP_TypeDef *pBMP, int32_t rawPressure)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)pBMP->calibration.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)pBMP->calibration.dig_p6;
	var2 = var2 + ((var1 * (int64_t)pBMP->calibration.dig_p5) << 17);
	var2 = var2 + (((int64_t)pBMP->calibration.dig_p4) << 35);
	var1 = ((var1 * var1 * (int64_t)pBMP->calibration.dig_p3) >> 8) + ((var1 * (int64_t)pBMP->calibration.dig_p2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)pBMP->calibration.dig_p1) >> 33;
	if(var1 == 0)
		return 0;

	p = 1048576 - rawPressure;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) pBMP->calibration.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)pBMP->calibration.dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)pBMP->calibration.dig_p7) << 4);
	return (uint32_t)p;
}

/*----------------------------------------------------------------------------*/
/**
  * @brief	Compensate humidity
  * @param	pBMP: pointer to the BME struct
  * @retval	None
  */
static uint32_t compensate_humidity(BMP_TypeDef *pBMP, int32_t rawHumidity)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (pBMP->calibration.t_fine - ((int32_t)76800));
	v_x1_u32r = (((((rawHumidity << 14) - (((int32_t)pBMP->calibration.dig_h4) << 20) - (((int32_t)pBMP->calibration.dig_h5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)pBMP->calibration.dig_h6)) >> 10) * (((v_x1_u32r * ((int32_t)pBMP->calibration.dig_h3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)pBMP->calibration.dig_h2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)pBMP->calibration.dig_h1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r >> 12);
}
