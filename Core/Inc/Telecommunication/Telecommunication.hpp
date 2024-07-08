#pragma once

#include <Rocket/AvionicsValues.hpp>
#include <Telecommunication/ITelecommunicationDriver.hpp>
#include <Telecommunication/GCSCommand.hpp>

class Telecommunication
{
public:
	Telecommunication(ITelecommunicationDriver* driver);
	~Telecommunication() {};

	void sendData(AvionicsValues data);
	void readCommand();
	GCSCommand getCommand();
private:
	ITelecommunicationDriver* driver;
};
