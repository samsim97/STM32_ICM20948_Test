#pragma once

#include <Sensors/Magnetometer/MagnetometerValues.hpp>
#include <Sensors/SensorState.hpp>

class Magnetometer
{
public:
	MagnetometerValues getValues();
	void fillData();
	SensorState getState();
};
