#pragma once

#include <HardwareDriver/BN220/NMEAMessage.hpp>

#define GLL_MESSAGE_TALKER_ID_FIELD_NUM 0
#define GLL_LATITUDE_FIELD_NUM 1
#define GLL_NORTH_SOUTH_INDICATOR_FIELD_NUM 2
#define GLL_LONGITUDE_FIELD_NUM 3
#define GLL_EAST_WEST_INDICATOR_FIELD_NUM 4
#define GLL_UTC_TIME_FIELD_NUM 5
#define GLL_STATUS_FIELD_NUM 6
#define GLL_POS_MODE_FIELD_NUM 7
#define GLL_CHECKSUM_FIELD_NUM 8
#define GLL_CARRIAGE_RETURN_LINE_FEED_FIELD_NUM 9

class GLLMessage : public NMEAMessage
{
public:
	GLLMessage() = default;
	virtual ~GLLMessage() = default;
	virtual void parse(const std::string& message) override;
	virtual size_t getMessageSize() override;

	static const size_t emptySize = sizeof("$GNGLL,,,,,,V,N*7A") - 1; // - 1 to remove \0 at end of const char*
};

