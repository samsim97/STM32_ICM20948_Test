#pragma once

#include <Sensors/Altimeter/AltimeterValues.hpp>
#include <Sensors/SensorState.hpp>

class Altimeter
{
public:
	AltimeterValues getValues();
	void fillData();
	SensorState getState();
};
