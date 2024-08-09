#pragma once

#include <string>
#include <Sensors/GPS/GPSValues.hpp>

class NMEAMessage
{
public:
	NMEAMessage();
	virtual ~NMEAMessage() = default;

	std::string getMessageId() const;
	std::string getTalkerId() const;
	std::uint8_t getChecksum() const;
	GPSPosition getPosition() const;

	virtual void parse(const std::string& message) = 0;
	virtual size_t getMessageSize() = 0;

protected:
	std::string messageId;
	std::string talkerId;
	std::uint8_t checksum;
	GPSPosition position;
};
