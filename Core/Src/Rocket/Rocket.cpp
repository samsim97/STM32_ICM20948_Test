#include <Rocket/Rocket.hpp>

Rocket::Rocket(I2C_HandleTypeDef* i2cHandle, UART_HandleTypeDef* uartHandleGPS, UART_HandleTypeDef* uartHandleXBEE)
{
	// Drivers -- Boards
	icm20948Driver = new ICM20948(i2cHandle);
	//bmp388Driver = new BMP388(i2cHandle);
	//bn220Driver = new BN220(uartHandleGPS);
	//xbeeDriver = new XBEE(uartHandleXBEE);

	// Sensors
	accelerometer = new Accelerometer(icm20948Driver);
	//altimeter = new Altimeter(bmp388Driver); // BMP388
	//gps = new GPS(bn220Driver); // GPS Driver
	gyroscope = new Gyroscope(icm20948Driver);

	// Telecom
	//telecommunication = new Telecommunication()
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
	test = 1;*/

	uint8_t testBuffer[5] = "Test";
	telecommunication->sendData(testBuffer, 5);
	telecommunication->getCommand();
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





