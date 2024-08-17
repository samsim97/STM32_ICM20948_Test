#include <Devices/Storage/Storage.hpp>


Storage::Storage(IStorageDriver* driver)
{
	this->driver = driver;
}

void Storage::saveData(uint8_t* data, uint16_t size)
{
	driver->write(data, size);
}

bool Storage::readData(uint8_t* buffer, uint16_t size)
{
	return driver->read(buffer, size);
}
