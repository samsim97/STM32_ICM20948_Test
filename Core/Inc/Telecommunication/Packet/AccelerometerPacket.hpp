#pragma once

#include <stdint.h>
#include <Sensors/Accelerometer/AccelerometerValues.hpp>

union AccelerometerPacket
{
	struct
	{
		uint16_t timeStamp;
		AccelerometerValues accelerometerValues;
	};
	uint8_t data[sizeof(timeStamp) + sizeof(AccelerometerValues)];
};
