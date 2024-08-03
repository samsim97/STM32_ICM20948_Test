#pragma once

#include "stm32f4xx_hal.h"

#include <stdint.h>

class Thermocouple
{
public:
	Thermocouple(ADC_HandleTypeDef* adcHandle, uint8_t channel);
	~Thermocouple() {};

	float getTemperature();
private:
	uint8_t channel;
	ADC_HandleTypeDef* adcHandle;

	const float referenceVoltage = 3.3f; // May need to change
	const float tempAt1kOhms = 24.0f;
	const float referenceResistance = 1000.0f;

	void configChannel();
};
