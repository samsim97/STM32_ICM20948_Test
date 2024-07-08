#pragma once

#include <Sensors/Gyroscope/Gyroscope.hpp>

Gyroscope::Gyroscope(IGyroscopeDriver* driver)
{
	this->driver = driver;
}


GyroscopeValues Gyroscope::getValues()
{
	return driver->getGyroscopeValues();
}

void Gyroscope::fillData()
{
	driver->readGyroscope();
}

SensorState Gyroscope::getState()
{
	return SensorState::SENSOR_OK;
}




