#pragma once

#include <stdint.h>
#include <Sensors/GPS/Direction.hpp>

#define GPS_VALUES_LENGTH 16

// SEE FORMAT NORD-SUD
struct CoordinateAxis
{
	uint16_t direction;
	uint16_t degrees;
	float minutes;
};

struct GPSPosition
{
	CoordinateAxis latitude;
	CoordinateAxis longitude;
};

union GPSValues
{
	GPSPosition gpsPosition;
	uint8_t values[GPS_VALUES_LENGTH];
};
