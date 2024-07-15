#pragma once

#include <stdint.h>
#include <Telecommunication/GCSCommand.hpp>

class ITelecommunicationDriver
{
public:
	virtual void sendData(uint8_t* data, uint8_t size) = 0;
	virtual void fetchData(uint8_t* buffer, uint8_t size) = 0;
	virtual GCSCommand getCommand() = 0;
};
