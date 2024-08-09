#pragma once

#include <string>

#include <HardwareDriver/BN220/NMEAMessage.hpp>
#include <HardwareDriver/BN220/GLLMessage.hpp>
#include <HardwareDriver/BN220/GGAMessage.hpp>

//const char* possibleMessageCodes[] = { "GLL", "GGA" };

class NMEAMessageFactory {

public:
	static NMEAMessage* createMessage(const std::string& messageId);

};

