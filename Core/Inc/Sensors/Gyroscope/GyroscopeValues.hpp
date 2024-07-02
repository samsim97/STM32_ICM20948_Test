#pragma once

union GyroscopeValues
{
	struct
	{
		float x_degPerSec;
		float y_degPerSec;
		float z_degPerSec;
	};
	float values_degPerSec[3];
};
