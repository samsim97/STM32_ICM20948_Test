#pragma once

#include <stdint.h>
#include <Devices/Thermocouple/ThermocoupleValues.hpp>
#include <Rocket/RocketDefines.hpp>

union ThermocouplePacket
{
	struct
	{
		uint8_t packetHeaderId;
		uint16_t timeStamp_cs;
		ThermocoupleValues thermocoupleValues[THERMOCOUPLE_AMOUNT];
	};
	uint8_t data[sizeof(packetHeaderId) + sizeof(timeStamp_cs) + (sizeof(ThermocoupleValues) * THERMOCOUPLE_AMOUNT)];
};
