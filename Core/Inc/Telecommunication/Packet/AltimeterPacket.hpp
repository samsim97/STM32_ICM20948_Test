#pragma once

#include <stdint.h>
#include <Sensors/Altimeter/AltimeterValues.hpp>

union AltimeterPacket
{
	struct
	{
		uint8_t packetHeaderId;
		uint16_t timeStamp_cs;
		AltimeterValues gyroscopeValues;
	};
	uint8_t data[sizeof(packetHeaderId) + sizeof(timeStamp_cs) + sizeof(AltimeterValues)];
};
