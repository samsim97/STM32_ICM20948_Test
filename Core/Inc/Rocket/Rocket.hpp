#pragma once


#include <Sensors/Accelerometer/Accelerometer.hpp>
#include <Sensors/Altimeter/Altimeter.hpp>
#include <Sensors/GPS/GPS.hpp>
#include <Sensors/Gyroscope/Gyroscope.hpp>
#include <Sensors/Magnetometer/Magnetometer.hpp>
#include <Sensors/Thermometer/Thermometer.hpp>

#include <Devices/SmokeBomb.hpp>
#include <Devices/Thermocouple.hpp>

#include <HardwareDriver/ICM20948/ICM20948.hpp>

class Rocket
{
public:
	Rocket(I2C_HandleTypeDef* i2chandle);
	~Rocket() {};

private:
	// Sensors
	Accelerometer* accelerometer;
	Altimeter* altimeter;
	GPS* gps;
	Gyroscope* gyroscope;
	Magnetometer* magnetometer;
	Thermometer* thermometer;

	// Devices
	SmokeBomb* smokeBomb;
	Thermocouple* thermocouple;

	// Drivers -- Boards
	ICM20948* icm20948Driver;
};
