#include <HardwareDriver/BN220/NMEAMessage.hpp>

NMEAMessage::NMEAMessage() {
	this->messageId = "";
	this->talkerId = "";
	this->checksum = 0;
	this->position = GPSPosition();
}

std::string NMEAMessage::getMessageId() const {
	return this->messageId;
}

std::string NMEAMessage::getTalkerId() const {
	return this->talkerId;
}

std::uint8_t NMEAMessage::getChecksum() const {
	return this->checksum;
}

GPSPosition NMEAMessage::getPosition() const {
	return this->position;
}
