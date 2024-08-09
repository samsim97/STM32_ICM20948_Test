#include <HardwareDriver/BN220/BN220.hpp>

BN220::BN220(UART_HandleTypeDef* uartHandle)
{
	this->uartHandle = uartHandle;
	gpsMessageParser = new GPSParser();
}

void BN220::readGPS()
{
	//int returnCode = HAL_UART_Receive(uartHandle, buffer, sizeof(buffer) - 1, 2000);

	//buffer = "$GPGGA,123456.00,4807.038,N,01131.000,E,1,12,1.0,545.4,M,46.9,M,,*47";
	memcpy(buffer, "$GPGGA,123456.00,4807.038,N,01131.000,E,1,12,1.0,545.4,M,46.9,M,,*47\r\n", strlen("$GPGGA,123456.00,4807.038,N,01131.000,E,1,12,1.0,545.4,M,46.9,M,,*47\r\n") + 1);

	updateGPSDataFromBuffer();
}

GPSValues BN220::getGPSValues()
{
	return gpsValues;
}

void BN220::updateGPSDataFromBuffer()
{
	std::string strBuffer(reinterpret_cast<char*>(buffer), BUFFER_LENGTH);
	gpsMessageParser->parse(strBuffer);

	//GPSPosition test = gpsMessageParser->getMessages()[0]->getPosition();

	//GPSPosition test2 = gpsMessageParser->getMessages()[0]->getPosition();
	gpsValues.gpsPosition = gpsMessageParser->getMessages()[0]->getPosition(); // HOW MANY MESSAGES DO WE REALLY WANNA GET IN ONE GO ? 1 SEEMS OK
}





