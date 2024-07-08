#pragma once

#include <Sensors/GPS/GPSValues.hpp>
#include <Sensors/GPS/IGPSDriver.hpp>
#include <Sensors/SensorState.hpp>

class GPS
{
public:
	GPS(IGPSDriver* driver);
	~GPS() {};

	GPSValues getValues();
	void fillData();
	SensorState getState();
};
