#pragma once

#include <Devices/Thermocouple/ThermocoupleValues.hpp>

#include <Sensors/Accelerometer/AccelerometerValues.hpp>
#include <Sensors/Gyroscope/GyroscopeValues.hpp>
#include <Sensors/Altimeter/AltimeterValues.hpp>
#include <Sensors/GPS/GPSValues.hpp>

struct StoredDataValues
{
	uint16_t accelerometerTimeStamp_cs;
	AccelerometerValues accelerometerValues;
	uint16_t altimeterTimeStamp_cs;
	AltimeterValues altimeterValues;
	uint16_t gyroscopeTimeStamp_cs;
	GyroscopeValues gyroscopeValues;
	uint16_t gpsTimeStamp_cs;
	GPSValues gpsValues;
	uint16_t thermocoupleTimeStamp_cs;
	ThermocoupleValues thermocoupleValues[4];
};

union StoredData
{
	StoredDataValues values;
	uint8_t data[sizeof(values)];
};
