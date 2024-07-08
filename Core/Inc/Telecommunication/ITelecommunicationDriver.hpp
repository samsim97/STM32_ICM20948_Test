#pragma once

#include <stdint.h>
#include <Telecommunication/GCSCommand.hpp>

class ITelecommunicationDriver
{
public:
	virtual void sendData() = 0;
	virtual void fetchData() = 0;
	virtual GCSCommand getCommand() = 0;
};
