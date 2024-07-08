#pragma once

union ThermocoupleValues
{
	struct
	{
		float temperature_C;
	};
	float values[1];
};
