#pragma once

#include <stdint.h>
#include <Sensors/Accelerometer/AccelerometerValues.hpp>

class AccelerometerData
{
public:
	AccelerometerData(AccelerometerValues accelerometerValues);

	uint16_t values[3];
};
