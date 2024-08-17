#pragma once

#include <HardwareDriver/STMFlash/STMFlash.hpp>

class Storage
{
public:
	Storage(IStorageDriver* driver);

	void saveData(uint8_t* data, uint16_t size);
	bool readData(uint8_t* buffer, uint16_t size);
private:
	IStorageDriver* driver;
};
