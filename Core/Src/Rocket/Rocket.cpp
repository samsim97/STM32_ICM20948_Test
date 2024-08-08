#include <Rocket/Rocket.hpp>

Rocket::Rocket(I2C_HandleTypeDef* i2cHandle, UART_HandleTypeDef* uartHandleXBEE, UART_HandleTypeDef* uartHandleGPS, ADC_HandleTypeDef* adcHandle)
{
	// Drivers -- Boards
	icm20948Driver = new ICM20948(i2cHandle);
	bmp388Driver = new BMP388(i2cHandle);
	bn220Driver = new BN220(uartHandleGPS);
	xbeeDriver = new XBEE(uartHandleXBEE);

	// Sensors
	accelerometer = new Accelerometer(icm20948Driver);
	altimeter = new Altimeter(bmp388Driver); // BMP388
	gps = new GPS(bn220Driver); // GPS Driver
	gyroscope = new Gyroscope(icm20948Driver);

	// Telecom
	telecommunication = new Telecommunication(xbeeDriver);

	// Devices
	smokeBomb = new SmokeBomb();

	thermocouple[0] = new Thermocouple(adcHandle, 10); // 10 => adc channel, put in const
	thermocouple[1] = new Thermocouple(adcHandle, 11);
	thermocouple[2] = new Thermocouple(adcHandle, 12);
	thermocouple[3] = new Thermocouple(adcHandle, 13);

	// FOR TEST ONLY
	currentFlightStage = FlightStage::ASCENDING;
}

void Rocket::initDrivers()
{
	icm20948Driver->init();
	bmp388Driver->init();
}

void Rocket::init()
{
	initDrivers();
	timeSinceLaunch_ms = HAL_GetTick();
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
	uint32_t accelFillTime_ms = accelerometer->fillData();
	uint32_t gyroFillTime_ms = gyroscope->fillData();
	uint32_t gpsFillTime_ms = gps->fillData();
	uint32_t altiFillTime_ms = altimeter->fillData();

	AccelerometerValues accelerometerValues = accelerometer->getValues();
	GyroscopeValues gyroscopeValues = gyroscope->getValues();
	GPSValues gpsValues = gps->getValues();
	AltimeterValues altimeterValues = altimeter->getValues();

	AccelerometerPacket accelerometerPacket = {accelFillTime_ms, accelerometerValues};
	AltimeterPacket altimeterPacket = {altiFillTime_ms, altimeterValues};
	GyroscopePacket gyroscopePacket = {gyroFillTime_ms, gyroscopeValues};
	GPSPacket gpsPacket = {gpsFillTime_ms, gpsValues};

	// Fragment data sending to reduce error rate
	telecommunication->sendData(accelerometerPacket.data, sizeof(AccelerometerPacket));
	telecommunication->sendData(altimeterPacket.data, sizeof(AltimeterPacket));
	telecommunication->sendData(gyroscopePacket.data, sizeof(GyroscopePacket));
	telecommunication->sendData(gpsPacket.data, sizeof(GPSPacket));

	/*uint8_t testBuffer[] = "Test";
	uint8_t receivedCommand[4] = {0};
	telecommunication->sendData(testBuffer, sizeof(testBuffer));
	telecommunication->fetchData(receivedCommand, sizeof(receivedCommand));
	//currentCommand = telecommunication->getCommand();
	if (receivedCommand[0] == 'c')
	{
		smokeBomb->ignite();
	}
	HAL_Delay(50);*/

	//altimeter->fillData();
	//altimeter->getValues();
	//ThermocoupleValues thermocoupleValues[THERMOCOUPLE_AMOUNT] = {0};
	//thermocoupleValues[3].temperature_C = thermocouple[3]->getTemperature();
	/*for (uint8_t i = 0; i < THERMOCOUPLE_AMOUNT; i++)
	{
		thermocoupleValues[i] = thermocouple[i]->getTemperature();
	}*/
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





