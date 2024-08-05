#include <HardwareDriver/BMP388/BMP388.hpp>

BMP388::BMP388(I2C_HandleTypeDef* i2cHandle)
{
	this->i2cHandle = i2cHandle;
}

void BMP388::init()
{
	dev_addr = BMP3_ADDR_I2C_SEC;
	//dev.read = (bmp3_read_fptr_t)SensorAPI_I2Cx_Read;
	//dev.write = (bmp3_write_fptr_t)SensorAPI_I2Cx_Write;
	dev.intf = BMP3_I2C_INTF;
	//dev.delay_us = bmp3_delay_us;
	dev.intf_ptr = &dev_addr;
	dev.dummy_byte = 0x0;

	rslt = bmp3_init(&dev);

	settings.int_settings.drdy_en = BMP3_ENABLE;
	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;

	settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
	settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
	settings.odr_filter.odr = BMP3_ODR_100_HZ;

	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
	                 BMP3_SEL_DRDY_EN;

	rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
	//bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

	settings.op_mode = BMP3_MODE_NORMAL;
	rslt = bmp3_set_op_mode(&settings, &dev);
	//bmp3_check_rslt("bmp3_set_op_mode", rslt);
}

void BMP388::readAltimeter()
{
	rslt = bmp3_get_status(&status, &dev);
	//bmp3_check_rslt("bmp3_get_status", rslt);

	/* Read temperature and pressure data iteratively based on data ready interrupt */
	if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE))
	{
		/*
	    * First parameter indicates the type of data to be read
	    * BMP3_PRESS_TEMP : To read pressure and temperature data
	    * BMP3_TEMP       : To read only temperature data
	    * BMP3_PRESS      : To read only pressure data
	    */
	    rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
	    //double sealevelpressure_hpa = 1013.25;
	    double sealevelpressure_hpa = 1022.25;
	    double altitude = 0.0;
	    float atmospheric = data.pressure / 100.0F;
		altitude = 44330.0 * (1.0 - std::pow(atmospheric / sealevelpressure_hpa, 0.1903));

	    //bmp3_check_rslt("bmp3_get_sensor_data", rslt);

	    /* NOTE : Read status register again to clear data ready interrupt status */
	    rslt = bmp3_get_status(&status, &dev);
	    //bmp3_check_rslt("bmp3_get_status", rslt);

	    loop = loop + 1;
	}

	HAL_Delay(5000);
}

AltimeterValues BMP388::getAltimeterValues()
{
	return altimeterValues;
}

int8_t BMP388::SensorAPI_I2Cx_Read(uint8_t subaddress, uint8_t *pBuffer, uint16_t ReadNumbr, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t devAddress = dev_addr << 1;

	// send register address
	HAL_I2C_Master_Transmit(i2cHandle, devAddress, &subaddress, 1, 1000);
	HAL_I2C_Master_Receive(i2cHandle, devAddress, pBuffer, ReadNumbr, 1000);
	return 0;
}

int8_t BMP388::SensorAPI_I2Cx_Write(uint8_t subaddress, uint8_t *pBuffer, uint16_t WriteNumbr, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t devAddress = dev_addr << 1;

	GTXBuffer[0] = subaddress;
	memcpy(&GTXBuffer[1], pBuffer, WriteNumbr);

	// send register address
	HAL_I2C_Master_Transmit(i2cHandle, devAddress, GTXBuffer, WriteNumbr+1, 1000);
	return 0;
}

void BMP388::bmp3_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}
