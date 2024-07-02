#pragma once

#include <stdint.h>

class SmokeBomb
{
public:
	SmokeBomb(uint8_t outputPinNumber);
	~SmokeBomb() {};

	void ignite();
private:
	uint8_t outputPin;
};
