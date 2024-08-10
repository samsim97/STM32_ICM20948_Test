#pragma once

#include <Sensors/Accelerometer/Accelerometer.hpp>

Accelerometer::Accelerometer(IAccelerometerDriver* driver)
{
	this->driver = driver;
	calibrationOffsets = {0, 0, 0};
	//calibrate(20);
}


AccelerometerValues Accelerometer::getValues()
{
	return driver->getAccelerometerValues();
}

uint32_t Accelerometer::fillData()
{
	driver->readAccelerometer();
	return HAL_GetTick();
}

void Accelerometer::calibrate(int16_t sampleSize)
{
	AccelerometerValues sampledValues = {0, 0, 0};
	for (uint16_t i = 0; i < sampleSize;i++)
	{
		fillData();
		sampledValues.x_mg = sampledValues.x_mg + getValues().x_mg;
		sampledValues.y_mg = sampledValues.y_mg + getValues().y_mg;
		sampledValues.z_mg = sampledValues.z_mg + getValues().z_mg;
	}

	calibrationOffsets.x_mg = sampledValues.x_mg / sampleSize;
	calibrationOffsets.y_mg = sampledValues.y_mg / sampleSize;
	calibrationOffsets.z_mg = sampledValues.z_mg / sampleSize;
}

SensorState Accelerometer::getState()
{
	return SensorState::SENSOR_OK;
}




