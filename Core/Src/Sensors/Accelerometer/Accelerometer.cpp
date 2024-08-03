#pragma once

#include <Sensors/Accelerometer/Accelerometer.hpp>

Accelerometer::Accelerometer(IAccelerometerDriver* driver)
{
	this->driver = driver;
	calibrationOffsets = {0.0f, 0.0f, 0.0f};
}


AccelerometerValues Accelerometer::getValues()
{
	return driver->getAccelerometerValues();
}

void Accelerometer::fillData()
{
	driver->readAccelerometer();
}

void Accelerometer::calibrate(int16_t sampleSize)
{
	AccelerometerValues sampledValues = {0.0f, 0.0f, 0.0f};
	for (uint16_t i = 0; i < sampleSize;i++)
	{
		fillData();
		sampledValues.x_g = sampledValues.x_g + getValues().x_g;
		sampledValues.y_g = sampledValues.y_g + getValues().y_g;
		sampledValues.z_g = sampledValues.z_g + getValues().z_g;
	}

	calibrationOffsets.x_g = sampledValues.x_g / (float)sampleSize;
	calibrationOffsets.y_g = sampledValues.y_g / (float)sampleSize;
	calibrationOffsets.z_g = sampledValues.z_g / (float)sampleSize;
}

SensorState Accelerometer::getState()
{
	return SensorState::SENSOR_OK;
}




