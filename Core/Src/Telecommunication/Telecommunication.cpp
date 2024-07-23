#include <Telecommunication/Telecommunication.hpp>

Telecommunication::Telecommunication(ITelecommunicationDriver* driver)
{
	this->driver = driver;
}


void Telecommunication::sendData(uint8_t* data, uint8_t size)
{
	driver->sendData(data, size);
}

GCSCommand Telecommunication::getCommand()
{
	driver->fetchData(currentCommand.values, sizeof(currentCommand));
	return currentCommand;
}
