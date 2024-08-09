#pragma once

#include <stdint.h>
#include <Sensors/GPS/Direction.hpp>

#define GPS_VALUES_LENGTH sizeof(GPSPosition)

// SEE FORMAT NORD-SUD
struct CoordinateAxis
{
	uint16_t degrees;
	float minutes;
	uint8_t direction;
};

struct GPSPosition
{
	CoordinateAxis latitude;
	CoordinateAxis longitude;
};

union GPSValues
{
	GPSPosition gpsPosition;
	uint16_t values[GPS_VALUES_LENGTH];
};
