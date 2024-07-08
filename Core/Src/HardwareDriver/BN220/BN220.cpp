#include <HardwareDriver/BN220/BN220.hpp>

BN220::BN220(UART_HandleTypeDef* uartHandle)
{
	this->uartHandle = uartHandle;
}

void BN220::readGPS()
{
	// gpsValues =
}

GPSValues BN220::getGPSValues()
{
	return gpsValues;
}





