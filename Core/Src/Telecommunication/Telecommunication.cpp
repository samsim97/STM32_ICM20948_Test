#include <Telecommunication/Telecommunication.hpp>

Telecommunication::Telecommunication(ITelecommunicationDriver* driver)
{

}


void Telecommunication::sendData(AvionicsValues data)
{

}

GCSCommand Telecommunication::getCommand()
{
	driver->getCommand();
}




