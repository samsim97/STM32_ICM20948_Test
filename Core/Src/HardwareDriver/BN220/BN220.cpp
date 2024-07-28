#include <HardwareDriver/BN220/BN220.hpp>

BN220::BN220(UART_HandleTypeDef* uartHandle)
{
	this->uartHandle = uartHandle;
}

void BN220::readGPS()
{
	int returnCode = HAL_UART_Receive(uartHandle, buffer, sizeof(buffer) - 1, 5000);


}

GPSValues BN220::getGPSValues()
{
	return gpsValues;
}





