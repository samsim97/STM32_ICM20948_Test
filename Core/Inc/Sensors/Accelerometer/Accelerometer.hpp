#pragma once

#include <Sensors/Accelerometer/AccelerometerValues.hpp>
#include <Sensors/SensorState.hpp>

class Accelerometer
{
public:
	AccelerometerValues getValues();
	void fillData();
	SensorState getState();
};
