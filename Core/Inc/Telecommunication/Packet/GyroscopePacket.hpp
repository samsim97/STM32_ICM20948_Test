#pragma once

#include <stdint.h>
#include <Sensors/Gyroscope/GyroscopeValues.hpp>

union GyroscopePacket
{
	struct
	{
		uint16_t packetHeaderId;
		uint16_t sensorHeaderId;
		uint16_t timeStamp;
		GyroscopeValues gyroscopeValues;
	};
	uint8_t data[sizeof(packetHeaderId) + sizeof(sensorHeaderId) + sizeof(timeStamp) + sizeof(GyroscopeValues)];
};
