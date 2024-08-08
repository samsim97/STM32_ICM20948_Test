#pragma once

#include <stdint.h>
#include <Sensors/Altimeter/AltimeterValues.hpp>

union AltimeterPacket
{
	struct
	{
		uint16_t timeStamp;
		AltimeterValues gyroscopeValues;
	};
	uint8_t data[sizeof(timeStamp) + sizeof(AltimeterValues)];
};
