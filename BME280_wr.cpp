#include "BME280_wr.h"

BME280::BME280(unsigned char address)
{
	m_bme280Dev.bus_write = i2CWrite;
	m_bme280Dev.bus_read = i2CRead;
	m_bme280Dev.dev_addr = address;
	m_bme280Dev.delay_msec = delayInMilliSec;
}

/* BME280::BME280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin):m_chipSelect(cspin),m_mosi(mosipin),m_miso(misopin),m_sck(sckpin)
{
	m_bme280Dev.bus_write = SPIWrite;
	m_bme280Dev.bus_read = SPIRead;
	m_bme280Dev.delay_msec = delayInMilliSec;	
} */

int8_t BME280::begin()
{
	int8_t rslt = BME280_ERROR;
	
	rslt = bme280_init(&m_bme280Dev);
	
	if(m_bme280Dev.chip_id != BME280_CHIP_ID)
	{
		rslt = BME280_ERROR;
	}

	return rslt;
}

void BME280::setSensorConfig()
{
	bme280_set_oversamp_humidity(humidityOverSampling); 
	bme280_set_oversamp_pressure(pressureOverSampling);
	bme280_set_oversamp_temperature(temperatureOverSampling);
	bme280_set_standby_durn(standbyDuration);
	bme280_set_filter(filterCoefficient);
	delay(SENSOR_CONFIGTIME);
}

void BME280::setPowerMode(uint8_t powerMode)
{
	bme280_set_power_mode(powerMode);	
}

void BME280::readSensorData(void)
{
	bme280_read_uncomp_temperature(&uncompTemperature);
	bme280_read_uncomp_pressure(&uncompPressure);
	bme280_read_uncomp_humidity(&uncompHumidity);

	temperature = bme280_compensate_temperature_double(uncompTemperature);
	pressure = bme280_compensate_pressure_double(uncompPressure);
	humidity = bme280_compensate_humidity_double(uncompHumidity);
}

/******************** Bridge functions for the Sensor API ************************************************************/
int8_t BME280::i2CRead(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint8_t len)
{
	int8_t comRes = BME280_ERROR;
	
	I2C.beginTransmission(devAddr);  
	I2C.write(regAddr);
	comRes = I2C.endTransmission();     
	I2C.requestFrom(devAddr, len, (byte)true);
	uint8_t index = 0;
	while (I2C.available())
	{
		regData[index] = I2C.read();
		index++;            
	}
	return comRes;	
}

int8_t BME280::i2CWrite(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint8_t len)
{
	int8_t comRes = BME280_ERROR;

	for(byte index = 0; index < len; index++)
	{  
		I2C.beginTransmission(devAddr);  
		I2C.write(regAddr + index);  
		byte datatobeWritten = regData[index];
		I2C.write(datatobeWritten);
		comRes =  I2C.endTransmission(true);    
	} 
	return comRes;
}

void BME280::delayInMilliSec(uint32_t period)
{
	delay(period);
}
