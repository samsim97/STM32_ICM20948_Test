#pragma once

#include <Sensors/Altimeter/IAltimeterDriver.hpp>
#include "stm32f4xx_hal.h"

class BMP388 : public IAltimeterDriver
{
public:
	BMP388(I2C_HandleTypeDef* i2chandle);
	~BMP388() {};

	void readAltimeter();
	AltimeterValues getAltimeterValues();
private:
	I2C_HandleTypeDef* i2cHandle;

	AltimeterValues altimeterValues;
};
