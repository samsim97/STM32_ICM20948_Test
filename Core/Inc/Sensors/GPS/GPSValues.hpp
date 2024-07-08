#pragma once

#include <stdint.h>

#define VALUES_LENGTH 6
// SEE FORMAT ?
struct CoordinateAxis
{
	uint16_t degrees;
	uint16_t minutes;
	uint16_t seconds;
};

struct GPSPosition
{
	CoordinateAxis latitude;
	CoordinateAxis longitude;
};

union GPSValues
{
	GPSPosition gpsPosition;
	uint16_t values[VALUES_LENGTH];
};
