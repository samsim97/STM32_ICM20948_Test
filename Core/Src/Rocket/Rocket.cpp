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

void Rocket::executeLaunching()
{

}

void Rocket::executeAscending()
{
	accelTimeStamp_ms = accelerometer->fillData();
	gyroTimeStamp_ms = gyroscope->fillData();
	gpsTimeStamp_ms = gps->fillData();
	altiTimeStamp_ms = altimeter->fillData();

	for (uint8_t i = 0; i < THERMOCOUPLE_AMOUNT; i++)
	{
		thermocoupleTimeStamp_ms[i] = thermocouple[i]->fillData();
	}

	AccelerometerValues accelerometerValues = accelerometer->getValues();
	GyroscopeValues gyroscopeValues = gyroscope->getValues();
	GPSValues gpsValues = gps->getValues();
	AltimeterValues altimeterValues = altimeter->getValues();

	ThermocoupleValues thermocoupleValues[THERMOCOUPLE_AMOUNT] = {0};

	for (uint8_t i = 0; i < THERMOCOUPLE_AMOUNT; i++)
	{
		thermocoupleValues[i] = thermocouple[i]->getValues();
	}

	AccelerometerPacket accelerometerPacket = {COM_HEADER_ID , ACCELEROMETER_HEADER_ID, static_cast<uint16_t>(accelTimeStamp_ms / 10), accelerometerValues};
	AltimeterPacket altimeterPacket = {COM_HEADER_ID, ALTIMETER_HEADER_ID, static_cast<uint16_t>(altiTimeStamp_ms / 10), altimeterValues};
	GyroscopePacket gyroscopePacket = {COM_HEADER_ID, GYROSCOPE_HEADER_ID, static_cast<uint16_t>(gyroTimeStamp_ms / 10), gyroscopeValues};
	GPSPacket gpsPacket = {COM_HEADER_ID, GPS_HEADER_ID, static_cast<uint16_t>(gpsTimeStamp_ms / 10), gpsValues};

	ThermocouplePacket thermocouplePacket = {COM_HEADER_ID, THERMOCOUPLE_HEADER_ID, static_cast<uint16_t>(gpsTimeStamp_ms / 10), {thermocoupleValues[0], thermocoupleValues[1], thermocoupleValues[2], thermocoupleValues[3]}};
	// Fragment data sending to reduce error rate
	telecommunication->sendData(accelerometerPacket.data, sizeof(accelerometerPacket.data));
	telecommunication->sendData(altimeterPacket.data, sizeof(altimeterPacket.data));
	telecommunication->sendData(gyroscopePacket.data, sizeof(gyroscopePacket.data));
	telecommunication->sendData(gpsPacket.data, sizeof(gpsPacket.data));

	telecommunication->sendData(thermocouplePacket.data, sizeof(thermocouplePacket.data));
	telecommunication->fetchData(currentCommand.values, sizeof(currentCommand));

	if (currentCommand.registerAddress == SMOKE_IGNITE_REGISTER && currentCommand.operation == 0x01 && currentCommand.value == 0x01)
	{
		smokeBomb->ignite();
	}

	uint8_t test = 0;

	/*uint8_t testBuffer[] = "Test";
	uint8_t receivedCommand[4] = {0};
	telecommunication->sendData(testBuffer, sizeof(testBuffer));

	//currentCommand = telecommunication->getCommand();
	if (receivedCommand[0] == 'c')
	{
		smokeBomb->ignite();
	}
	HAL_Delay(50);*/
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





