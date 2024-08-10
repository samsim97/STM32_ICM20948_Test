#pragma once

#include <stdint.h>
#include <Sensors/Altimeter/AltimeterValues.hpp>

union AltimeterPacket
{
	struct
	{
		uint16_t packetHeaderId;
		uint16_t timeStamp_cs;
		AltimeterValues altimeterValues;
	};
	uint8_t data[sizeof(packetHeaderId) + sizeof(timeStamp_cs) + sizeof(AltimeterValues)];
};
