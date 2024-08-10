#pragma once

// Packet Formats
#include <Telecommunication/Packet/AccelerometerPacket.hpp>
#include <Telecommunication/Packet/AltimeterPacket.hpp>
#include <Telecommunication/Packet/GyroscopePacket.hpp>
#include <Telecommunication/Packet/GPSPacket.hpp>

#include <Telecommunication/Packet/ThermocouplePacket.hpp>

#include <Telecommunication/ITelecommunicationDriver.hpp>
#include <Telecommunication/GCSCommand.hpp>

#define COM_HEADER_ID 0xA55A

#define ACCELEROMETER_HEADER_ID 0xA510U
#define ALTIMETER_HEADER_ID 0xA520U
#define GYROSCOPE_HEADER_ID 0xA530U
#define GPS_HEADER_ID 0xA540U

#define THERMOCOUPLE_HEADER_ID 0xA550U

class Telecommunication
{
public:
	Telecommunication(ITelecommunicationDriver* driver);
	~Telecommunication() {};

	void sendData(uint8_t* data, uint8_t size);
	void fetchData(uint8_t* data, uint8_t size);
	GCSCommand getCommand();
private:
	ITelecommunicationDriver* driver;
	GCSCommand currentCommand;
};
