#pragma once

#include <Sensors/Accelerometer/AccelerometerValues.hpp>
#include <Sensors/Accelerometer/IAccelerometerDriver.hpp>
#include <Sensors/SensorState.hpp>

class Accelerometer
{
public:
	Accelerometer(IAccelerometerDriver* driver);
	~Accelerometer() {};

	AccelerometerValues getValues();
	void fillData();
	SensorState getState();
private:
	IAccelerometerDriver* driver;
};
