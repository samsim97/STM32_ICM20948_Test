#include <Telecommunication/Telecommunication.hpp>

Telecommunication::Telecommunication(ITelecommunicationDriver* driver)
{
	this->driver = driver;
}


void Telecommunication::sendData(uint8_t* data, uint8_t size)
{
	uint8_t test = data[0];
	uint8_t test1 = data[1];
	uint8_t test2 = data[size - 1];
	uint8_t test3 = 0;
	driver->sendData(data, size);
}

void Telecommunication::fetchData(uint8_t* buffer, uint8_t size)
{
	driver->fetchData(buffer, size);
}

GCSCommand Telecommunication::getCommand()
{
	driver->fetchData(currentCommand.values, sizeof(currentCommand));
	return currentCommand;
}
