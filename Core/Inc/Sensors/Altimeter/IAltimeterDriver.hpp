#pragma once

#include <Sensors/Altimeter/AltimeterValues.hpp>
#include <Sensors/SensorState.hpp>

class IAltimeterDriver
{
public:
	virtual AltimeterValues getAltimeterValues() = 0;
	virtual void readAltimeter() = 0;
};
