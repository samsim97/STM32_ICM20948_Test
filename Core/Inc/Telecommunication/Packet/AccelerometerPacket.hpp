#pragma once

#include <stdint.h>
#include <Sensors/Accelerometer/AccelerometerValues.hpp>

union AccelerometerPacket
{
	struct
	{
		uint8_t packetHeaderId;
		uint16_t timeStamp_cs;
		AccelerometerValues accelerometerValues;
	};
	uint8_t data[sizeof(packetHeaderId) + sizeof(timeStamp_cs) + sizeof(AccelerometerValues)];
};
