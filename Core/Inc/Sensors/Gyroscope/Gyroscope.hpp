#pragma once

#include <Sensors/Gyroscope/GyroscopeValues.hpp>
#include <Sensors/Gyroscope/IGyroscopeDriver.hpp>
#include <Sensors/SensorState.hpp>

class Gyroscope
{
public:
	Gyroscope(IGyroscopeDriver* driver);
	~Gyroscope() {};

	GyroscopeValues getValues();
	void fillData();
	SensorState getState();

private:
	IGyroscopeDriver* driver;
};
