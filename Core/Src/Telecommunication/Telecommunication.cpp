#include <Telecommunication/Telecommunication.hpp>

Telecommunication::Telecommunication(ITelecommunicationDriver* driver)
{

}


void Telecommunication::sendData(AvionicsValues data)
{
	driver->sendData(data.values, sizeof(data));
}

GCSCommand Telecommunication::getCommand()
{
	return driver->getCommand();
}




