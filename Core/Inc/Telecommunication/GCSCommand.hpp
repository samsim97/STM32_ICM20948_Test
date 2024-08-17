#pragma once

#include <stdint.h>

union GCSCommand
{
	struct
	{
		uint16_t registerAddress;
		uint16_t operation;
		uint16_t value;
	};
	uint8_t values[6];
};
