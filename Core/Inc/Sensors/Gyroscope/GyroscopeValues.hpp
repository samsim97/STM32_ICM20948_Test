#pragma once

#include <stdint.h>

union GyroscopeValues
{
	struct
	{
		int16_t x_mdegPerSec;
		int16_t y_mdegPerSec;
		int16_t z_mdegPerSec;
	};
	int16_t values_degPerSec[3];
};
