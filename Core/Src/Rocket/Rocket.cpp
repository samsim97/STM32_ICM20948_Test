#pragma once

#include <Rocket/Rocket.hpp>

Rocket::Rocket(I2C_HandleTypeDef* i2chandle)
{
	// Drivers -- Boards
	icm20948Driver = new ICM20948(i2chandle);

	// Sensors
	accelerometer = new Accelerometer(icm20948Driver);
	//altimeter = Altimeter(); // BMP388
	//gps = GPS(); // GPS Driver
	gyroscope = new Gyroscope(icm20948Driver);
}

/*************************************/
/***            Sensors            ***/
/*************************************/





