#pragma once

#include <stdint.h>
#include <Sensors/GPS/Direction.hpp>

#define GPS_VALUES_LENGTH 6

// SEE FORMAT NORD-SUD
struct CoordinateAxis
{
	uint16_t degrees;
	uint16_t minutes;
	uint16_t seconds;
	Direction direction;
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
