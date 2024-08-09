#pragma once

#include <Sensors/GPS/GPSValues.hpp>
#include <Sensors/GPS/IGPSDriver.hpp>

#include <HardwareDriver/BN220/GLLMessage.hpp>
#include <HardwareDriver/BN220/GGAMessage.hpp>

#include <HardwareDriver/BN220/GPSParser.hpp>

#include "stm32f4xx_hal.h"

#include <string.h>

#define BUFFER_LENGTH 0x100U

class BN220: public IGPSDriver
{
public:
	BN220(UART_HandleTypeDef* uartHandle);
	~BN220() {};

	void readGPS();
	GPSValues getGPSValues();
private:
	UART_HandleTypeDef* uartHandle;
	GPSParser* gpsMessageParser;
	GPSValues gpsValues;

	uint8_t buffer[BUFFER_LENGTH] = {0};
	int receiveOKCount = 0;

	void updateGPSDataFromBuffer();
};
