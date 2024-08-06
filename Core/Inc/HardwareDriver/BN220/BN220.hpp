#pragma once

#include <Sensors/GPS/GPSValues.hpp>
#include <Sensors/GPS/IGPSDriver.hpp>

#include "stm32f4xx_hal.h"

#include <string.h>

class BN220: public IGPSDriver
{
public:
	BN220(UART_HandleTypeDef* uartHandle);
	~BN220() {};

	void readGPS();
	GPSValues getGPSValues();
private:
	UART_HandleTypeDef* uartHandle;
	GPSValues gpsValues;

	uint8_t buffer[256] = {0};
	int receiveOKCount = 0;

	const char* HEADER_CODE_GGA = "$GPGGA";
	const char* HEADER_CODE_GLL = "$GPGLL";

	GPSValues getDataFromBuffer();
};
