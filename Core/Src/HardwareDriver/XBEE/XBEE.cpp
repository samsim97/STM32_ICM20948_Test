#include <HardwareDriver/XBEE/XBEE.hpp>

XBEE::XBEE(UART_HandleTypeDef* uartHandle)
{
	this->uartHandle = uartHandle;
	enableXBEE();
}

void XBEE::sendData(uint8_t* data, uint8_t size)
{
	HAL_UART_Transmit(uartHandle, data, size, UART_COMMUNICATION_TIMOUT_MS);
}

void XBEE::fetchData(uint8_t* buffer, uint8_t size)
{
	HAL_UART_Receive(uartHandle, buffer, size, UART_COMMUNICATION_TIMOUT_MS);
}

void XBEE::enableXBEE()
{
	HAL_Delay(5000);
	uint8_t bit [] = "b";
	HAL_UART_Transmit (uartHandle,  bit, sizeof(bit), UART_COMMUNICATION_TIMOUT_MS);
	HAL_Delay(2500);
	HAL_UART_Transmit (uartHandle,  bit, sizeof(bit), UART_COMMUNICATION_TIMOUT_MS);
	HAL_Delay(2500);
	HAL_UART_Transmit (uartHandle,  bit, sizeof(bit), UART_COMMUNICATION_TIMOUT_MS);
	HAL_Delay(500);
}
