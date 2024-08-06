#pragma once

#include <Sensors/Altimeter/AltimeterValues.hpp>
#include <Sensors/Altimeter/IAltimeterDriver.hpp>
#include <Sensors/SensorState.hpp>

#include "stm32f4xx_hal.h"

class Altimeter
{
public:
	Altimeter(IAltimeterDriver* driver);
	~Altimeter() {};

	AltimeterValues getValues();
	uint32_t fillData();
	SensorState getState();
private:
	IAltimeterDriver* driver;
	SensorState sensorState;
};
