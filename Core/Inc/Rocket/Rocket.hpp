#pragma once


#include <Sensors/Accelerometer/Accelerometer.hpp>
#include <Sensors/Altimeter/Altimeter.hpp>
#include <Sensors/GPS/GPS.hpp>
#include <Sensors/Gyroscope/Gyroscope.hpp>
#include <Sensors/Magnetometer/Magnetometer.hpp>
#include <Sensors/Thermometer/Thermometer.hpp>

#include <Devices/SmokeBomb.hpp>
#include <Devices/Thermocouple.hpp>

class Rocket
{
public:
	Rocket();
	~Rocket() {};

private:
	// Sensors
	Accelerometer accelerometer;
	Altimeter altimeter;
	GPS gps;
	Gyroscope gyroscope;
	Magnetometer magnetometer;
	Thermometer thermometer;

	// Devices
	SmokeBomb smokeBomb;
	Thermocouple thermocouple;

	// Drivers -- Boards

};
