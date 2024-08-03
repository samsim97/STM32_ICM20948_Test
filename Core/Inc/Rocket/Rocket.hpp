#pragma once


#include <Sensors/Accelerometer/Accelerometer.hpp>
#include <Sensors/Altimeter/Altimeter.hpp>
#include <Sensors/GPS/GPS.hpp>
#include <Sensors/Gyroscope/Gyroscope.hpp>
#include <Sensors/Magnetometer/Magnetometer.hpp>
#include <Sensors/Thermometer/Thermometer.hpp>

#include <Devices/SmokeBomb.hpp>
#include <Devices/Thermocouple/Thermocouple.hpp>

#include <Telecommunication/Telecommunication.hpp>

#include <HardwareDriver/ICM20948/ICM20948.hpp>
#include <HardwareDriver/BMP388/BMP388.hpp>
#include <HardwareDriver/BN220/BN220.hpp>
#include <HardwareDriver/XBEE/XBEE.hpp>

#include <Rocket/FlightStage.hpp>

#define THERMOCOUPLE_AMOUNT 0x4U

class Rocket
{
public:
	Rocket(I2C_HandleTypeDef* i2cHandle, UART_HandleTypeDef* uartHandleXBEE, UART_HandleTypeDef* uartHandleGPS, ADC_HandleTypeDef* adcHandle);
	~Rocket() {};

	void initDrivers();
	void execute();

	FlightStage getCurrentFlightStage();
	void setCurrentFlightStage(FlightStage flightStage);
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
	Thermocouple* thermocouple[THERMOCOUPLE_AMOUNT];

	// Telecommunication
	Telecommunication* telecommunication;

	// Drivers -- Boards
	ICM20948* icm20948Driver;
	//BMP388* bmp388Driver;
	//BN220* bn220Driver;
	XBEE* xbeeDriver;

	// State Machine
	FlightStage currentFlightStage;
	GCSCommand currentCommand;
	void executeIntializing();
	void executeAscending();
};
