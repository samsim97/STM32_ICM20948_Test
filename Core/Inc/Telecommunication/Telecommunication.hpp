#pragma once

#include <Telecommunication/ITelecommunicationDriver.hpp>

class Telecommunication
{
public:
	Telecommunication(ITelecommunicationDriver* driver);
	~Telecommunication() {};
};
