#pragma once

#include <Sensors/Altimeter/AltimeterValues.hpp>
#include <Sensors/Altimeter/IAltimeterDriver.hpp>
#include <Sensors/SensorState.hpp>

class Altimeter
{
public:
	Altimeter(IAltimeterDriver* driver);
	~Altimeter() {};

	AltimeterValues getValues();
	void fillData();
	SensorState getState();
private:
	IAltimeterDriver* driver;
	SensorState sensorState;
};
