#ifndef __BME280_WR_H__
#define __BME280_WR_H__

#include <utility/bme280.h>
#include <Wire.h>
#include <SPI.h>
#include "Arduino.h"

#if defined(ARDUINO_SAM_DUE)
#define I2C		Wire1	//Define which I2C bus is used. Wire1 for the Arduino Due
#else
#define I2C             Wire    //Or Wire
#endif

#define SENSOR_CONFIGTIME	5

class BME280 
{
public:
	/* Constructors */
	BME280(unsigned char address = 0x76); //Constructor used to pass the I2C device address 
				       //Since there are two device addresses for the bme280 sensor,
				       //which can be configured by the SDO pin
					   //Default I2C device address is 0x76 (BME280 SDO PIN connected to GND)
	//BME280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);
	
	/*Variables to store compensated Sensor data */
	double temperature;
	double pressure;
	double humidity;
	
	/*Variables to store raw (uncompensated Sensor data)*/
	int32_t uncompTemperature;
	int32_t uncompPressure;
	int32_t uncompHumidity;
	
	/*************************************************
	*	OVER SAMPLING DEFINITIONS  
	*******************************************************
	*	value       |  Over sampling 
	* ------------------|-------------------------
	*	0x00        | BME280_OVERSAMP_SKIPPED
	*	0x01        | BME280_OVERSAMP_1X 
	*	0x02        | BME280_OVERSAMP_2X 
	*	0x03        | BME280_OVERSAMP_4X 
	*	0x04        | BME280_OVERSAMP_8X 
	*       0x05        | BME280_OVERSAMP_16X
	*******************************************************
	*	STANDBY DURATION DEFINITIONS  
	*******************************************************
	*	value       |   Standby duration 
	* ------------------|-------------------------
	*	0x00        | BME280_STANDBY_TIME_1_MS   
	*	0x01        | BME280_STANDBY_TIME_63_MS  
	*	0x02        | BME280_STANDBY_TIME_125_MS 
	*	0x03        | BME280_STANDBY_TIME_250_MS 
	*	0x04        | BME280_STANDBY_TIME_500_MS 
	*	0x05        | BME280_STANDBY_TIME_1000_MS
	*	0x06        | BME280_STANDBY_TIME_10_MS  
	*	0x07        | BME280_STANDBY_TIME_20_MS
	*******************************************************
	*	FILTER COEFFICIENT DEFINITIONS 
	*******************************************************
	*	value       |  Filter coefficient
	* ------------------|-------------------------
	*	0x00        | BME280_FILTER_COEFF_OFF
	*	0x01        | BME280_FILTER_COEFF_2
	*	0x02        | BME280_FILTER_COEFF_4
	*	0x03        | BME280_FILTER_COEFF_8
	*	0x04        | BME280_FILTER_COEFF_16
	********************************************************/
	/*Sensor settings*/
	uint8_t temperatureOverSampling;
	uint8_t pressureOverSampling;
	uint8_t humidityOverSampling;
	uint8_t standbyDuration;
	uint8_t filterCoefficient;

	/*******************************************************************************************
	*Description: Function with the initialization
	*Input Parameters: None
	*Return Parameter: None
	*******************************************************************************************/
	int8_t begin();
	
	/*******************************************************************************************
	*Description: This function is used to set the sensor settings
	*Input Parameters: None
	*Return Parameter: None
	*******************************************************************************************/	
	void setSensorConfig();
	
	/*******************************************************************************************
	*Description: This function is used to set the sensor power mode
	*
	*Input Parameters: powerMode - The value of power mode
	*
	*       value           |    powerMode
	* ----------------------|------------------
	*	0x00            | BME280_SLEEP_MODE
	*	0x01 and 0x02   | BME280_FORCED_MODE
	*	0x03            | BME280_NORMAL_MODE
	*
	*******************************************************************************************/
	void setPowerMode(uint8_t powerMode);
	
	/*******************************************************************************************
	*Description: This function is used to read the sensor data.
	*Input Parameters: none
	*Return Parameter: None
	*******************************************************************************************/
	void readSensorData(void);

	/******************** Bridge functions for the Sensor API *********************************/
	static int8_t i2CRead(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint8_t len);
		
	static int8_t i2CWrite(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint8_t len);

	static void delayInMilliSec(uint32_t period);	
	
private:
	struct bme280_t m_bme280Dev;
	//int8_t m_chipSelect, m_mosi, m_miso, m_sck;
};

#endif