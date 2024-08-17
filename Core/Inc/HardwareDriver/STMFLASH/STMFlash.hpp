#pragma once

#include <Devices/Storage/IStorageDriver.hpp>
#include <Devices/Storage/StoredData.hpp>
#include "stm32f4xx_hal.h"

class STMFlash : public IStorageDriver
{
public:
	STMFlash();
	~STMFlash() {}

	void write(uint8_t* data, uint32_t size);
	bool read(uint8_t* buffer, uint32_t size);

private:
	//static const uint32_t BUFFER_STACK_SIZE = 0x10000;

	//static const uint8_t START_SECTOR = FLASH_SECTOR_7;
	static const uint32_t START_ADDRESS = 0x08040000; // For 256 Kib storage

	static const uint32_t STORAGE_SIZE_BYTES = 0x40000; // 256 Kib

	//static const uint8_t START_SECTOR = FLASH_SECTOR_7;
	static const uint32_t END_ADDRESS = START_ADDRESS + STORAGE_SIZE_BYTES;

	//uint8_t* buffer[BUFFER_STACK_SIZE];
	static const uint8_t START_BYTE = 0x80;

	static const uint8_t STORED_DATA_SIZE_BIT = (sizeof(StoredData) + sizeof(START_BYTE));
	uint32_t currentWriteAddress;
	uint32_t currentReadAddress;

	void resetReadAddress();

	void programByte(uint8_t data);
	void programHalfWord(uint16_t data);
	void programWord(uint32_t data);
	void programDoubleWord(uint64_t data);

	uint8_t readByte(uint32_t address);
	bool checkAddress(uint32_t address);

	void writeFlash(uint8_t* data, uint32_t size);
	bool readFlash(uint8_t* buffer, uint32_t size);

	void findCurrentAddress();
	void clearMemory();
};
