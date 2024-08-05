#include <Devices/Thermocouple/Thermocouple.hpp>

Thermocouple::Thermocouple(ADC_HandleTypeDef* adcHandle, uint8_t channel)
{
	this->channel = channel;
	this->adcHandle = adcHandle;
}

float Thermocouple::getTemperature()
{
	float temperature = 0.0f;

	configChannel();

	HAL_ADC_Start(adcHandle);

	HAL_ADC_PollForConversion(adcHandle, HAL_MAX_DELAY);

	uint32_t adcValue = HAL_ADC_GetValue(adcHandle);

	// Apply formula for temp
	float adcValue_volt = ((float)adcValue / 4095.0) * referenceVoltage;

	float thermistanceValue = ((adcValue_volt / referenceVoltage) * referenceResistance) / ((adcValue_volt / referenceVoltage) + 1);

	temperature = (1000.0f / thermistanceValue) * 24.0f; // MAY NEED TO CHANGE
	return temperature;
}

void Thermocouple::configChannel()
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(adcHandle, &sConfig);
}



