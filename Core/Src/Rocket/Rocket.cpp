#include <Rocket/Rocket.hpp>

Rocket::Rocket(I2C_HandleTypeDef* i2cHandle, UART_HandleTypeDef* uartHandleXBEE, UART_HandleTypeDef* uartHandleGPS, ADC_HandleTypeDef* adcHandle)
{
	// Drivers -- Boards
	icm20948Driver = new ICM20948(i2cHandle);
	bmp388Driver = new BMP388(i2cHandle);
	bn220Driver = new BN220(uartHandleGPS);
	xbeeDriver = new XBEE(uartHandleXBEE);

	// Drivers -- STM
	stmFlashDriver = new STMFlash();

	// Sensors
	accelerometer = new Accelerometer(icm20948Driver);
	altimeter = new Altimeter(bmp388Driver); // BMP388
	gps = new GPS(bn220Driver); // GPS Driver
	gyroscope = new Gyroscope(icm20948Driver);

	// Telecom
	telecommunication = new Telecommunication(xbeeDriver);

	// Devices
	smokeBomb = new SmokeBomb();

	storage = new Storage(stmFlashDriver);

	thermocouple[0] = new Thermocouple(adcHandle, 10); // 10 => adc channel, put in const
	thermocouple[1] = new Thermocouple(adcHandle, 11);
	thermocouple[2] = new Thermocouple(adcHandle, 12);
	thermocouple[3] = new Thermocouple(adcHandle, 13);

	// FOR TEST ONLY
	currentFlightStage = FlightStage::ASCENDING;

	isSaveActivated = true;
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
	StoredData storedData;

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

	AccelerometerPacket accelerometerPacket = {COM_HEADER_ID , ACCELEROMETER_HEADER_ID, static_cast<uint16_t>(accelTimeStamp_ms / 100), accelerometerValues};
	AltimeterPacket altimeterPacket = {COM_HEADER_ID, ALTIMETER_HEADER_ID, static_cast<uint16_t>(altiTimeStamp_ms / 100), altimeterValues};
	GyroscopePacket gyroscopePacket = {COM_HEADER_ID, GYROSCOPE_HEADER_ID, static_cast<uint16_t>(gyroTimeStamp_ms / 100), gyroscopeValues};
	GPSPacket gpsPacket = {COM_HEADER_ID, GPS_HEADER_ID, static_cast<uint16_t>(gpsTimeStamp_ms / 100), gpsValues};

	ThermocouplePacket thermocouplePacket = {COM_HEADER_ID, THERMOCOUPLE_HEADER_ID, static_cast<uint16_t>(thermocoupleTimeStamp_ms[0] / 100), {thermocoupleValues[0], thermocoupleValues[1], thermocoupleValues[2], thermocoupleValues[3]}};
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

	if (isSaveActivated)
	{
		storedData.values.accelerometerTimeStamp_cs = static_cast<uint16_t>(accelTimeStamp_ms / 100);
		storedData.values.accelerometerValues = accelerometerValues;
		storedData.values.altimeterTimeStamp_cs = static_cast<uint16_t>(altiTimeStamp_ms / 100);
		storedData.values.altimeterValues = altimeterValues;
		storedData.values.gyroscopeTimeStamp_cs = static_cast<uint16_t>(gyroTimeStamp_ms / 100);
		storedData.values.gyroscopeValues = gyroscopeValues;
		storedData.values.gpsTimeStamp_cs = static_cast<uint16_t>(gpsTimeStamp_ms / 100);
		storedData.values.gpsValues = gpsValues;
		storedData.values.thermocoupleTimeStamp_cs = static_cast<uint16_t>(thermocoupleTimeStamp_ms[3] / 100);
		storedData.values.thermocoupleValues[0] = thermocoupleValues[0];
		storedData.values.thermocoupleValues[1] = thermocoupleValues[1];
		storedData.values.thermocoupleValues[2] = thermocoupleValues[2];
		storedData.values.thermocoupleValues[3] = thermocoupleValues[3];

		storage->saveData(storedData.data, sizeof(storedData));
	}

	// COMMENT
	currentCommand.registerAddress = DATA_FETCHING_REGISTER;
	currentCommand.operation = 0x00;
	currentCommand.value = 0;

	if (currentCommand.registerAddress == DATA_FETCHING_REGISTER && currentCommand.operation == 0x00 && currentCommand.value == 0x00)
	{
		StoredData dataStored;

		while (storage->readData(dataStored.data, sizeof(dataStored)))
		{
			AccelerometerPacket accelerometerPacket = {COM_HEADER_ID , ACCELEROMETER_HEADER_ID, dataStored.values.accelerometerTimeStamp_cs , dataStored.values.accelerometerValues};
			AltimeterPacket altimeterPacket = {COM_HEADER_ID, ALTIMETER_HEADER_ID, dataStored.values.altimeterTimeStamp_cs, dataStored.values.altimeterValues};
			GyroscopePacket gyroscopePacket = {COM_HEADER_ID, GYROSCOPE_HEADER_ID, dataStored.values.gyroscopeTimeStamp_cs, dataStored.values.gyroscopeValues};
			GPSPacket gpsPacket = {COM_HEADER_ID, GPS_HEADER_ID, dataStored.values.gpsTimeStamp_cs, dataStored.values.gpsValues};

			ThermocouplePacket thermocouplePacket = {COM_HEADER_ID, THERMOCOUPLE_HEADER_ID, dataStored.values.thermocoupleTimeStamp_cs, {dataStored.values.thermocoupleValues[0], dataStored.values.thermocoupleValues[1], dataStored.values.thermocoupleValues[2], dataStored.values.thermocoupleValues[3]}};

			telecommunication->sendData(accelerometerPacket.data, sizeof(accelerometerPacket.data));
			telecommunication->sendData(altimeterPacket.data, sizeof(altimeterPacket.data));
			telecommunication->sendData(gyroscopePacket.data, sizeof(gyroscopePacket.data));
			telecommunication->sendData(gpsPacket.data, sizeof(gpsPacket.data));

			telecommunication->sendData(thermocouplePacket.data, sizeof(thermocouplePacket.data));
		}
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





