#pragma once

#include <stdint.h>

// This interface is temporary, DO NOT TOUCH ANY OF THE STORAGE CODE WITHOUT KNOWING WHAT YOU'RE DOING
class IStorageDriver
{
public:
	// Write to storage
	virtual void write(uint8_t* data, uint32_t size) = 0;

	// Read last not already read storage data
	virtual bool read(uint8_t* buffer, uint32_t size) = 0;
};
