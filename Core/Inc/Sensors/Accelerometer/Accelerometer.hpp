#pragma once

#include <Sensors/Accelerometer/AccelerometerValues.hpp>
#include <Sensors/Accelerometer/IAccelerometerDriver.hpp>
#include <Sensors/SensorState.hpp>

#include <stdint.h>
#include "stm32f4xx_hal.h"

class Accelerometer
{
public:
	Accelerometer(IAccelerometerDriver* driver);
	~Accelerometer() {};

	AccelerometerValues getValues();
	uint32_t fillData();
	void calibrate(int16_t sampleSize);
	SensorState getState();
private:
	IAccelerometerDriver* driver;
	AccelerometerValues calibrationOffsets;
};
