#pragma once

#include <stdint.h>
#include <Devices/Thermocouple/ThermocoupleValues.hpp>
#include <Rocket/RocketDefines.hpp>

union ThermocouplePacket
{
	struct
	{
		uint16_t timeStamp;
		ThermocoupleValues thermocoupleValues[THERMOCOUPLE_AMOUNT];
	};
	uint8_t data[sizeof(timeStamp) + (sizeof(ThermocoupleValues) * THERMOCOUPLE_AMOUNT)];
};
