#pragma once

#include <Telecommunication/ITelecommunicationDriver.hpp>

class XBEE : public ITelecommunicationDriver
{
public:
	void sendData(uint8_t* data, uint8_t size) = 0;
	void fetchData(uint8_t* buffer, uint8_t size) = 0;
	GCSCommand getCommand() = 0;
};
