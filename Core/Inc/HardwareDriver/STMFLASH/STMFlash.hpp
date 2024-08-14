#pragma once

#include <Devices/Storage/IStorageDriver.hpp>
#include "stm32f4xx_hal.h"

class STMFlash : public IStorageDriver
{
public:
	STMFlash();
	~STMFlash() {}

	void write(uint8_t* data, uint32_t size);
	void read(uint8_t* buffer, uint32_t size);

private:
	static const uint32_t BUFFER_STACK_SIZE = 0x10000;

	static const uint32_t START_SECTOR = 0x08040000; // For 256 Kib storage
	static const uint32_t STORAGE_SIZE = 0x40000; // 256 Kib
	static const uint32_t END_SECTOR = START_SECTOR + STORAGE_SIZE;
	//static const uint32_t SECTOR_SIZE = 0x;

	uint8_t* buffer[BUFFER_STACK_SIZE];
	uint8_t currentSector;
};
