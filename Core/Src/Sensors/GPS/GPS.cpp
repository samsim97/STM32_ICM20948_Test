#include <Sensors/GPS/GPS.hpp>

#include "stm32f4xx_hal.h"

GPS::GPS(IGPSDriver* driver)
{
	this->driver = driver;
}

GPSValues GPS::getValues()
{
	return driver->getGPSValues();
}

uint32_t GPS::fillData()
{
	driver->readGPS();
	return HAL_GetTick();
}

SensorState GPS::getState()
{
	return sensorState;
}




