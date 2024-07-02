#pragma once

#include <Sensors/SensorState.hpp>

class Thermometer
{
public:
	float getValue_C();
	void fillData();
	SensorState getState();
};
