#pragma once

#include <stdint.h>
#include <Sensors/GPS/GPSValues.hpp>

union GPSPacket
{
	struct
	{
		uint16_t timeStamp;
		GPSValues gyroscopeValues;
	};
	uint8_t data[sizeof(timeStamp) + sizeof(GPSValues)];
};
