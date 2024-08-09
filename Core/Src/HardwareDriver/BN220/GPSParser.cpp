#include <HardwareDriver/BN220/GPSParser.hpp>

GPSParser::GPSParser() {
	this->messages = new NMEAMessage*[10];
}

void GPSParser::parse(const std::string& data) {
	if (data.empty()) {
		return;
	}
  const char firstChar = '$';
	std::uint16_t messageCount = 0;
  //const char* messageCodes[] = { "GLL", "GGA" };
  for (int i = 0; i < data.size(); i++) {
    if (data[i] == firstChar) {
			std::string messageCode = data.substr(i + 3, 3);
			NMEAMessage* message = NMEAMessageFactory::createMessage(messageCode);
			if (message == nullptr) {
				// Log bits and pieces
				continue;
			}
			const bool messageIsComplete = data.find("\r\n", i) != std::string::npos;
			if (!messageIsComplete) {
				// Log bits and pieces
				continue;
			}
			message->parse(data.substr(i, data.find("\r\n", i) - i));
			i = data.find("\r\n", i) + 1;
			messages[messageCount++] = message;
		}
  }
}

NMEAMessage** GPSParser::getMessages() const {
  return messages;
}
