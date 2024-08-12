#pragma once

#include <Devices/Thermocouple/ThermocoupleValues.hpp>

#include <Sensors/Accelerometer/AccelerometerValues.hpp>
#include <Sensors/Gyroscope/GyroscopeValues.hpp>
#include <Sensors/Altimeter/AltimeterValues.hpp>
#include <Sensors/GPS/GPSValues.hpp>

struct StoredDataValues
{
	AccelerometerValues accelerometerValues;
	AltimeterValues altimeterValues;
	GyroscopeValues gyroscopeValues;
	GPSValues gpsValues;
	ThermocoupleValues thermocoupleValues;
};

union StoredData
{
	StoredDataValues values;
	uint8_t data[STORED_DATA_SIZE_BYTES];
};
