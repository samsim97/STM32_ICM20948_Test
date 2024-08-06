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

uint32_t Gyroscope::fillData()
{
	driver->readGyroscope();
	return HAL_GetTick();
}

SensorState Gyroscope::getState()
{
	return SensorState::SENSOR_OK;
}




