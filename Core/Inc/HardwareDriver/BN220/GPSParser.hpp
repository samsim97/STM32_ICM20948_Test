#pragma once

#include <string>

#include <HardwareDriver/BN220/NMEAMessage.hpp>
#include <HardwareDriver/BN220/NMEAMessageFactory.hpp>

class GPSParser {

public:
	GPSParser();
	~GPSParser() = default;

	void parse(const std::string& data);
	NMEAMessage** getMessages() const;

private:
	NMEAMessage** messages;

	void clearMessages();
};

