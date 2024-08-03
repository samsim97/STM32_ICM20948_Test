#pragma once

#include <stdint.h>

class Thermocouple
{
public:
	Thermocouple(uint8_t channel);
	~Thermocouple() {};

	float getTemperature();
private:
	uint8_t channel;
};
