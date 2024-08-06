#include <Sensors/Altimeter/Altimeter.hpp>

Altimeter::Altimeter(IAltimeterDriver* driver)
{
	this->driver = driver;
}


AltimeterValues Altimeter::getValues()
{
	return driver->getAltimeterValues();
}

uint32_t Altimeter::fillData()
{
	driver->readAltimeter();
	return HAL_GetTick();
}

SensorState Altimeter::getState()
{
	return sensorState;
}




