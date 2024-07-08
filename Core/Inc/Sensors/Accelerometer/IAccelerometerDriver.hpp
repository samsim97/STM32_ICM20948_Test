#pragma once

#include <Sensors/Accelerometer/AccelerometerValues.hpp>
#include <Sensors/SensorState.hpp>

class IAccelerometerDriver
{
public:
	virtual AccelerometerValues getAccelerometerValues() = 0;
	virtual void readAccelerometer() = 0;
};
