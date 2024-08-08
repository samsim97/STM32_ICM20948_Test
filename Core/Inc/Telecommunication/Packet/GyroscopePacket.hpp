#pragma once

#include <stdint.h>
#include <Sensors/Gyroscope/GyroscopeValues.hpp>

union GyroscopePacket
{
	struct
	{
		uint16_t timeStamp;
		GyroscopeValues gyroscopeValues;
	};
	uint8_t data[sizeof(timeStamp) + sizeof(GyroscopeValues)];
};
