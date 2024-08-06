#pragma once

#include <Sensors/Gyroscope/GyroscopeValues.hpp>
#include <Sensors/Gyroscope/IGyroscopeDriver.hpp>
#include <Sensors/SensorState.hpp>

#include <stdint.h>
#include "stm32f4xx_hal.h"

class Gyroscope
{
public:
	Gyroscope(IGyroscopeDriver* driver);
	~Gyroscope() {};

	GyroscopeValues getValues();
	uint32_t fillData();
	SensorState getState();

private:
	IGyroscopeDriver* driver;
};
