#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

#include <Devices/Thermocouple/ThermocoupleValues.hpp>

class Thermocouple
{
public:
	Thermocouple(ADC_HandleTypeDef* adcHandle, uint8_t channel);
	~Thermocouple() {};

	uint32_t fillData();
	ThermocoupleValues getValues();
private:
	uint8_t channel;
	ADC_HandleTypeDef* adcHandle;

	ThermocoupleValues thermocoupleValues;

	const float referenceVoltage = 3.3f; // May need to change
	const float tempAt10kOhms = 24.0f;
	const float referenceResistance = 10000.0f;

	void configChannel();
};
