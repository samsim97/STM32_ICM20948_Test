#pragma once

#include <Sensors/GPS/GPSValues.hpp>
#include <Sensors/SensorState.hpp>

class GPS
{
public:
	GPSValues getValues();
	void fillData();
	SensorState getState();
};
