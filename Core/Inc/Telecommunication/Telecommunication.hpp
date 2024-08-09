#pragma once

// Packet Formats
#include <Telecommunication/Packet/AccelerometerPacket.hpp>
#include <Telecommunication/Packet/AltimeterPacket.hpp>
#include <Telecommunication/Packet/GyroscopePacket.hpp>
#include <Telecommunication/Packet/GPSPacket.hpp>

#include <Telecommunication/ITelecommunicationDriver.hpp>
#include <Telecommunication/GCSCommand.hpp>

#define COM_HEADER_ID 0x69U

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
