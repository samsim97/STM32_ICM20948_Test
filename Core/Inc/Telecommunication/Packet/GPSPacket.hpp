#pragma once

#include <stdint.h>
#include <Sensors/GPS/GPSValues.hpp>

union GPSPacket
{
	struct
	{
		uint8_t packetHeaderId;
		uint16_t timeStamp_cs;
		GPSValues gyroscopeValues;
	};
	uint8_t data[sizeof(packetHeaderId) + sizeof(timeStamp_cs) + sizeof(GPSValues)];
};
