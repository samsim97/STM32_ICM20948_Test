#pragma once

#include <Telecommunication/ITelecommunicationDriver.hpp>
#include <stdint.h>
#include "stm32f4xx_hal.h"

class XBEE : public ITelecommunicationDriver
{
public:
	XBEE(UART_HandleTypeDef* uartHandle);
	~XBEE() {};

	void sendData(uint8_t* data, uint8_t size);
	void fetchData(uint8_t* buffer, uint8_t size);

private:
	UART_HandleTypeDef* uartHandle;
	const uint16_t UART_COMMUNICATION_TIMOUT_MS = 200U;
};
