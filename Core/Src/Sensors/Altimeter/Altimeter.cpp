#include <Sensors/Altimeter/Altimeter.hpp>

Altimeter::Altimeter(IAltimeterDriver* driver)
{
	this->driver = driver;
}


AltimeterValues Altimeter::getValues()
{
	return driver->getAltimeterValues();
}

void Altimeter::fillData()
{
	driver->readAltimeter();
}

SensorState Altimeter::getState()
{
	return sensorState;
}




