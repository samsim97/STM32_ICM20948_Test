#pragma once

// Sensors
#include <Sensors/Accelerometer/AccelerometerValues.hpp>
#include <Sensors/Altimeter/AltimeterValues.hpp>
#include <Sensors/GPS/GPSValues.hpp>
#include <Sensors/Gyroscope/GyroscopeValues.hpp>
#include <Sensors/Magnetometer/MagnetometerValues.hpp>
#include <Sensors/Thermometer/ThermometerValues.hpp>

// Devices
#include <Devices/Thermocouple/ThermocoupleValues.hpp>

struct SensorValues
{
	AccelerometerValues accelerometerValues;
	AltimeterValues altimeterValues;
	GPSValues gpsValues;
	GyroscopeValues gyroscopeValues;
	MagnetometerValues magnetometerValues;
	ThermometerValues thermometerValues;
};

struct DeviceValues
{
	ThermocoupleValues thermocoupleValues;
};


#define TOTAL_SIZE_BYTE sizeof(SensorValues) + sizeof(DeviceValues)

union AvionicsValues
{
	struct
	{
		SensorValues sensorValues;
		DeviceValues deviceValues;
	};
	uint8_t values[TOTAL_SIZE_BYTE];
};
