#include <Rocket/Rocket.hpp>

Rocket::Rocket(I2C_HandleTypeDef* i2chandle)
{
	// Drivers -- Boards
	icm20948Driver = new ICM20948(i2chandle);
	icm20948Driver->init();

	// Sensors
	accelerometer = new Accelerometer(icm20948Driver);
	//altimeter = Altimeter(); // BMP388
	//gps = GPS(); // GPS Driver
	gyroscope = new Gyroscope(icm20948Driver);

	// FOR TEST ONLY
	currentFlightStage = FlightStage::ASCENDING;
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
	accelerometer->fillData();
	uint16_t test= 0;

	AccelerometerValues accelValues = accelerometer->getValues();
	test = 1;
}

void Rocket::getCurrentFlightStage()
{

}

void Rocket::setCurrentFlightStage(FlightStage flightStage)
{

}

/*************************************/
/***            Sensors            ***/
/*************************************/





