#pragma once

union ThermometerValues
{
	struct
	{
		float temperature_C;
	};
	float values[1];
};
