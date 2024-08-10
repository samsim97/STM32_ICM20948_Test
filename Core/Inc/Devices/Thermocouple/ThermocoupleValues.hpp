#pragma once

union ThermocoupleValues
{
	struct
	{
		uint16_t temperature_cC;
	};
	uint16_t values[1];
};
