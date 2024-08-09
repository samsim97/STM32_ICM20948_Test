#pragma once

#include <HardwareDriver/BN220/NMEAMessage.hpp>

class GGAMessage : public NMEAMessage
{
public:
	GGAMessage() = default;
	virtual ~GGAMessage() = default;
	virtual void parse(const std::string& message) override;
	virtual size_t getMessageSize() override;

	static const size_t emptySize = sizeof("$GNGGA,,,,,,0,00,99.99,,,,,,*56") - 1; // - 1 to remove \0 at end of const char*
};

