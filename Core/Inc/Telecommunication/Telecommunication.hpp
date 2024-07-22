#pragma once

#include <Rocket/AvionicsValues.hpp>
#include <Telecommunication/ITelecommunicationDriver.hpp>
#include <Telecommunication/GCSCommand.hpp>

class Telecommunication
{
public:
	Telecommunication(ITelecommunicationDriver* driver);
	~Telecommunication() {};

	void sendData(uint8_t* data, uint8_t size);
	GCSCommand getCommand();
private:
	ITelecommunicationDriver* driver;
	GCSCommand currentCommand;
};
