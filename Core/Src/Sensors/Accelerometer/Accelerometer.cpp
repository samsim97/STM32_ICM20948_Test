#pragma once

#include <Sensors/Accelerometer/Accelerometer.hpp>

Accelerometer::Accelerometer(IAccelerometerDriver* driver)
{
	this->driver = driver;
}


AccelerometerValues Accelerometer::getValues()
{
	return driver->getAccelerometerValues();
}

void Accelerometer::fillData()
{
	driver->readAccelerometer();
}

SensorState Accelerometer::getState()
{
	return SensorState::SENSOR_OK;
}




