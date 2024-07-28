#pragma once

#include <stdint.h>

union GCSCommand
{
	struct
	{
		uint8_t registerAddress;
		uint8_t operation;
		uint8_t value;
		uint8_t endl;
	};
	uint8_t values[4];
};
