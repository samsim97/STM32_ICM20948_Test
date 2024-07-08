#pragma once

#include <Sensors/GPS/GPSValues.hpp>
#include <Sensors/SensorState.hpp>

class IGPSDriver
{
public:
	virtual GPSValues getGPSValues() = 0;
	virtual void readGPS() = 0;
};
