#include <HardwareDriver/BMP388/BMP388.hpp>

BMP388::BMP388(I2C_HandleTypeDef* i2cHandle)
{
	this->i2cHandle = i2cHandle;
}

void BMP388::readAltimeter()
{
	//altimeterValues =
}

AltimeterValues BMP388::getAltimeterValues()
{
	return altimeterValues;
}




