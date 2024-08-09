#include <HardwareDriver/BN220/NMEAMessageFactory.hpp>

NMEAMessage* NMEAMessageFactory::createMessage(const std::string& messageId)
{
	bool isMessageCodeValid = false;
	for (const char* messageCode : { "GLL", "GGA" }) {
		if (messageId == messageCode) {
			isMessageCodeValid = true;
			break;
		}
	}
	if (!isMessageCodeValid) {
		return nullptr;
	}
	if (messageId == "GLL") {
		return new GLLMessage();
	} else if (messageId == "GGA") {
		return new GGAMessage();
	}
	return nullptr;
}
