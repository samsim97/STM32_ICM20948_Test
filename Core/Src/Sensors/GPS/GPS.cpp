#include <Sensors/GPS/GPS.hpp>

GPS::GPS(IGPSDriver* driver)
{
	this->driver = driver;
}

GPSValues GPS::getValues()
{
	return driver->getGPSValues();
}

void GPS::fillData()
{
	driver->readGPS();
}

SensorState GPS::getState()
{
	return sensorState;
}




