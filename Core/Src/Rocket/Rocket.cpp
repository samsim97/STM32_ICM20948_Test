#include <Rocket/Rocket.hpp>

Rocket::Rocket(I2C_HandleTypeDef* i2cHandle, UART_HandleTypeDef* uartHandleXBEE, UART_HandleTypeDef* uartHandleGPS, ADC_HandleTypeDef* adcHandle)
{
	// Drivers -- Boards
	icm20948Driver = new ICM20948(i2cHandle);
	//bmp388Driver = new BMP388(i2cHandle);
	//bn220Driver = new BN220(uartHandleGPS);
	xbeeDriver = new XBEE(uartHandleXBEE);

	// Sensors
	accelerometer = new Accelerometer(icm20948Driver);
	//altimeter = new Altimeter(bmp388Driver); // BMP388
	//gps = new GPS(bn220Driver); // GPS Driver
	gyroscope = new Gyroscope(icm20948Driver);

	// Telecom
	telecommunication = new Telecommunication(xbeeDriver);

	// Devices
	smokeBomb = new SmokeBomb();
	// FOR TEST ONLY
	currentFlightStage = FlightStage::ASCENDING;
}

void Rocket::initDrivers()
{
	icm20948Driver->init();
}

void Rocket::execute()
{
	switch(currentFlightStage)
	{
		case FlightStage::INITIALIZING:
			executeIntializing();
			break;
		case FlightStage::ASCENDING:
			executeAscending();
			break;
	}
}

void Rocket::executeIntializing()
{

}

void Rocket::executeAscending()
{
	/*accelerometer->fillData();
	gyroscope->fillData();

	uint16_t test= 0;

	AccelerometerValues accelValues = accelerometer->getValues();
	GyroscopeValues gyroValues = gyroscope->getValues();
	test = 1;

	telecommunication->sendData(accelValues.values_g, sizeof(AccelerometerValues));*/

	uint8_t testBuffer[] = "Test";
	uint8_t receivedCommand[4] = {0};
	telecommunication->sendData(testBuffer, sizeof(testBuffer));
	telecommunication->fetchData(receivedCommand, sizeof(receivedCommand));
	//currentCommand = telecommunication->getCommand();
	if (receivedCommand[0] == 'c')
	{
		smokeBomb->ignite();
	}
	HAL_Delay(50);
}

FlightStage Rocket::getCurrentFlightStage()
{
	return currentFlightStage;
}

void Rocket::setCurrentFlightStage(FlightStage flightStage)
{
	currentFlightStage = flightStage;
}

/*************************************/
/***            Sensors            ***/
/*************************************/





