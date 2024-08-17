#pragma once


#include <Sensors/Accelerometer/Accelerometer.hpp>
#include <Sensors/Altimeter/Altimeter.hpp>
#include <Sensors/GPS/GPS.hpp>
#include <Sensors/Gyroscope/Gyroscope.hpp>
#include <Sensors/Magnetometer/Magnetometer.hpp>
#include <Sensors/Thermometer/Thermometer.hpp>

#include <Devices/SmokeBomb/SmokeBomb.hpp>
#include <Devices/Thermocouple/Thermocouple.hpp>
#include <Devices/Thermocouple/ThermocoupleValues.hpp>
#include <Devices/Storage/Storage.hpp>

#include <Telecommunication/Telecommunication.hpp>

#include <HardwareDriver/ICM20948/ICM20948.hpp>
#include <HardwareDriver/BMP388/BMP388.hpp>
#include <HardwareDriver/BN220/BN220.hpp>
#include <HardwareDriver/XBEE/XBEE.hpp>
#include <HardwareDriver/STMFlash/STMFlash.hpp>

#include <Rocket/FlightStage.hpp>
#include <Rocket/RocketDefines.hpp>
#include <Rocket/RegisterMap.hpp>

class Rocket
{
public:
	Rocket(I2C_HandleTypeDef* i2cHandle, UART_HandleTypeDef* uartHandleXBEE, UART_HandleTypeDef* uartHandleGPS, ADC_HandleTypeDef* adcHandle);
	~Rocket() {};

	void init();
	void execute();

	FlightStage getCurrentFlightStage();
	void setCurrentFlightStage(FlightStage flightStage);
private:
	// Rocket variables
	uint32_t timeSinceLaunch_ms = 0;

	uint32_t accelTimeStamp_ms = 0;
	uint32_t gyroTimeStamp_ms = 0;
	uint32_t gpsTimeStamp_ms = 0;
	uint32_t altiTimeStamp_ms = 0;

	uint32_t thermocoupleTimeStamp_ms[THERMOCOUPLE_AMOUNT] = {0};

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
	Storage* storage;

	// Telecommunication
	Telecommunication* telecommunication;

	// Drivers -- Boards
	ICM20948* icm20948Driver;
	BMP388* bmp388Driver;
	BN220* bn220Driver;
	XBEE* xbeeDriver;

	// Drivers -- STM
	STMFlash* stmFlashDriver;

	// State Machine
	FlightStage currentFlightStage;
	GCSCommand currentCommand;

	bool isSaveActivated;
	bool dataGatheringActivated;

	void executeIntializing();
	void executeLaunching();
	void executeAscending();

	void initDrivers();
};
