#pragma once

#include <stdint.h>
#include "stm32f4xx_hal.h"

class SmokeBomb
{
public:
	SmokeBomb();
	~SmokeBomb() {};

	void ignite();
private:
	uint8_t outputPin;
};
