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
	driver->fetchData(currentCommand.values, sizeof(currentCommand));
	return currentCommand;
}




