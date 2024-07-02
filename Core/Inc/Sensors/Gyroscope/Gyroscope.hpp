#pragma once

#include <Sensors/Gyroscope/GyroscopeValues.hpp>
#include <Sensors/SensorState.hpp>

class Gyroscope
{
public:
	GyroscopeValues getValues();
	void fillData();
	SensorState getState();
};
