#pragma once

#include <Sensors/Gyroscope/GyroscopeValues.hpp>
#include <Sensors/SensorState.hpp>

class IGyroscopeDriver
{
public:
	virtual GyroscopeValues getGyroscopeValues() = 0;
	virtual void readGyroscope() = 0;
};
