#pragma once

#include <HardwareDriver/BN220/NMEAMessage.hpp>

class GGAMessage : public NMEAMessage
{
public:
	GGAMessage() = default;
	virtual ~GGAMessage() = default;
	virtual void parse(const std::string& message) override;
	virtual size_t getMessageSize() override;
};

