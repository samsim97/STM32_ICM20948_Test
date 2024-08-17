#pragma once

#include <stdint.h>

#include <Rocket/AvionicsValues.hpp>
#include <Sensors/Gyroscope/GyroscopeValues.hpp>

#define SMOKE_IGNITE_REGISTER 0x10
#define DATA_FETCHING_REGISTER 0x80
#define DATA_CLEAR_REGISTER 0x81

struct Registers
{
	uint8_t versionRegister; // 0x0, R
	uint8_t resetRegister; // 0x1, RW
	SensorValues sensorRegisters; // 0xn-0xm, R
};

union RegisterMap
{
	Registers registers;
	uint8_t values[sizeof(Registers)];
};
