#include <HardwareDriver/BN220/GLLMessage.hpp>

void GLLMessage::parse(const std::string& message) {
	std::string delimiter = ",";
	uint8_t currentField = 0;
	std::string token;
	size_t previousIndex = 0;
	size_t currentIndex = 0;

	std::string tempMessage = message;
	while ((currentIndex = tempMessage.find(delimiter)) != std::string::npos) {
		token = tempMessage.substr(0, currentIndex);
		tempMessage = tempMessage.substr(currentIndex + 1);
		switch (currentField)
		{
		case 0:
			talkerId = token.substr(0, 2);
			messageId = token.substr(2, 3);
			break;
		case 1:
			position.latitude.degrees = std::stoul(token.substr(0, 2));
			position.latitude.minutes = std::stof(token.substr(2, currentIndex));
			break;
		case 2:
			position.latitude.direction = token[0];
			break;
		case 3:
			position.longitude.degrees = std::stoul(token.substr(0, 3));
			position.longitude.minutes = std::stof(token.substr(3, currentIndex));
			break;
		case 4:
			position.longitude.direction = token[0];
		case 5:
			return;
		default:
			break;
		}
		previousIndex = currentIndex;
		currentField++;
	}
}

size_t GLLMessage::getMessageSize() {
	return -1;
}
