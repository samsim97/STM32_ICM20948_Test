#pragma once

#include <stdint.h>
#include <Sensors/GPS/GPSValues.hpp>

union GPSPacket
{
	struct
	{
		uint16_t packetHeaderId;
		uint16_t sensorHeaderId;
		uint16_t timeStamp_cs;
		GPSValues gpsValues;
	};
	uint8_t data[sizeof(packetHeaderId) + sizeof(sensorHeaderId) + sizeof(timeStamp_cs) + sizeof(GPSValues)];
};
