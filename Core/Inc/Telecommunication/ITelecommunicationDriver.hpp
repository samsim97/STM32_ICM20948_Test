#pragma once

#include <stdint.h>

class ITelecommunicationDriver
{
public:
	virtual void sendData() = 0;
	virtual void fetchData() = 0;
};
