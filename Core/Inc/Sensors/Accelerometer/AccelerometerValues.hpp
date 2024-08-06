#pragma once

#include <stdint.h>

union AccelerometerValues
{
	struct
	{
		float x_g;
		float y_g;
		float z_g;
	};
	float values_g[3];
};
