#pragma once

#include <Sensors/Altimeter/IAltimeterDriver.hpp>
#include "stm32f4xx_hal.h"
#include <string.h>
#include <cmath>

#include <HardwareDriver/Bmp388/bmp3.h>

class BMP388 : public IAltimeterDriver
{
public:
	BMP388(I2C_HandleTypeDef* i2chandle);
	~BMP388() {};

	struct bmp3_dev dev;

	void init();
	void readAltimeter();
	AltimeterValues getAltimeterValues();

private:
	I2C_HandleTypeDef* i2cHandle;

	AltimeterValues altimeterValues;

	// REFACTOR
	uint8_t GTXBuffer[512], GRXBuffer[2048];
	uint8_t dev_addr = 0;
	int8_t rslt;
	uint8_t loop = 0;
	uint16_t settings_sel;

	struct bmp3_data data = { 0 };
	struct bmp3_settings settings = { 0 };
	struct bmp3_status status = { { 0 } };

	int8_t SensorAPI_I2Cx_Read(uint8_t subaddress, uint8_t *pBuffer, uint16_t ReadNumbr, void *intf_ptr);

	int8_t SensorAPI_I2Cx_Write(uint8_t subaddress, uint8_t *pBuffer, uint16_t WriteNumbr, void *intf_ptr);

	void bmp3_delay_us(uint32_t period, void *intf_ptr);

	static int8_t ReadWrapper(uint8_t reg_addr, uint8_t* data, uint16_t len, void* intf_ptr);
};
