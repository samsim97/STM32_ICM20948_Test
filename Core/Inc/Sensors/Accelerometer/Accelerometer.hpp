#pragma once

#include <Sensors/Accelerometer/AccelerometerValues.hpp>
#include <Sensors/SensorState.hpp>

class Accelerometer
{
public:
	Accelerometer

	AccelerometerValues getValues();
	void fillData();
	SensorState getState();
private:

};
