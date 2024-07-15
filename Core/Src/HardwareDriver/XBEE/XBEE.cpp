#pragma once

#include <HardwareDriver/XBEE/XBEE.hpp>

XBEE::XBEE(UART_HandleTypeDef* uartHandle)
{
	this->uartHandle = uartHandle;
}

void XBEE::sendData(uint8_t* data, uint8_t size)
{
	HAL_UART_Transmit(uartHandle, data, size, UART_COMMUNICATION_TIMOUT_MS);
}

void XBEE::fetchData(uint8_t* buffer, uint8_t size)
{
	HAL_UART_Receive(uartHandle, buffer, size, UART_COMMUNICATION_TIMOUT_MS);
}




