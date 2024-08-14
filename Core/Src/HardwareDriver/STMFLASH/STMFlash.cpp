#include <HardwareDriver/STMFLASH/STMFlash.hpp>

STMFlash::STMFlash()
{
	currentSector = 0;
}

void STMFlash::write(uint8_t* data, uint32_t size)
{
	HAL_FLASH_Unlock();
}

void STMFlash::read(uint8_t* buffer, uint32_t size)
{

}





