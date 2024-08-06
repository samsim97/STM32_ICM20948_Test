#include <HardwareDriver/BN220/BN220.hpp>

BN220::BN220(UART_HandleTypeDef* uartHandle)
{
	this->uartHandle = uartHandle;
}

void BN220::readGPS()
{
	int returnCode = HAL_UART_Receive(uartHandle, buffer, sizeof(buffer) - 1, 2000);
	int test0 = 0;
	for (uint16_t i = 0; i < sizeof(buffer); i++)
	{
		uint8_t letter = buffer[i];
		int test1 = 0;
	}
	int test2 = 0;
}

GPSValues BN220::getGPSValues()
{
	return gpsValues;
}

GPSValues BN220::getDataFromBuffer()
{
	uint8_t delimiter = ',';

	/*buffer = "$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B";
	char *strValue = strtok((char*)buffer, ",");
	while (myPtr != NULL)
	{
	  printf("%s\n", strValue);
	  strValue = strtok(NULL, ",");
	}
	return 0;*/
}





