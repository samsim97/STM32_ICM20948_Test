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
	uint32_t fillData();
	SensorState getState();
private:
	IGPSDriver* driver;
	SensorState sensorState;
};
