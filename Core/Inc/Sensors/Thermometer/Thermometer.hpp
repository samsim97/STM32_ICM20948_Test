#pragma once

#include <Sensors/Thermometer/ThermometerValues.hpp>
#include <Sensors/SensorState.hpp>

class Thermometer
{
public:
	ThermometerValues getValue_C();
	void fillData();
	SensorState getState();
};
