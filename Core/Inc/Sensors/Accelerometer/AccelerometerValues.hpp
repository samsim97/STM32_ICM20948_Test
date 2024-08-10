#pragma once

#include <stdint.h>

union AccelerometerValues
{
	struct
	{
		int16_t x_mg;
		int16_t y_mg;
		int16_t z_mg;
	};
	int16_t values_mg[3];
};
