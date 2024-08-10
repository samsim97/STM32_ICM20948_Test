#pragma once

#include <stdint.h>

union AltimeterValues
{
	struct
	{
		uint16_t height_cm;
	};
	uint16_t values[1];
};
