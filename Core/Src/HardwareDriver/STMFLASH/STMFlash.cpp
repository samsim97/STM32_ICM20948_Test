#include <HardwareDriver/STMFLASH/STMFlash.hpp>

STMFlash::STMFlash()
{
	currentWriteAddress = START_ADDRESS;
	currentReadAddress = START_ADDRESS;

	//clearMemory();
}

void STMFlash::write(uint8_t* data, uint32_t size)
{
	writeFlash(data, size);
}

bool STMFlash::read(uint8_t* buffer, uint32_t size)
{
	return readFlash(buffer, size);
}

void STMFlash::resetReadAddress()
{
	currentReadAddress = START_ADDRESS;
}

void STMFlash::writeFlash(uint8_t* data, uint32_t size)
{
	HAL_StatusTypeDef resultUnlock = HAL_FLASH_Unlock();

	programByte(START_BYTE);
	for (uint16_t i = 0; i < size; i++)
	{
		programByte(data[i]);
	}

	HAL_StatusTypeDef resultLock = HAL_FLASH_Lock();
}

bool STMFlash::readFlash(uint8_t* buffer, uint32_t size)
{
	uint8_t tempValue = 0x00;
	for (uint32_t i = 0; i < size + 1; i++)
	{
		if (checkAddress(currentReadAddress))
		{
			tempValue = readByte(currentReadAddress);
			if ((START_ADDRESS - currentReadAddress) % (sizeof(StoredData) + 1) != 0)
			{
				buffer[i] = tempValue;
			}
			else
			{
				i--;
			}
			currentReadAddress++;
		}
		else
		{
			return false;
		}
	}
	return true;
}

uint8_t STMFlash::readByte(uint32_t address)
{
	if (checkAddress(address))
	{
		return *(volatile uint8_t*)address;
	}
	return 0;
}

void STMFlash::clearMemory()
{
	FLASH_Erase_Sector(FLASH_SECTOR_6, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_7, FLASH_VOLTAGE_RANGE_3);
}

bool STMFlash::checkAddress(uint32_t address)
{
	return address >= START_ADDRESS && address + 1 <= END_ADDRESS;
}

void STMFlash::findCurrentAddress()
{
	uint8_t value = 0;

	for (uint32_t address = START_ADDRESS; address < END_ADDRESS; address += STORED_DATA_SIZE_BIT)
	{
		value = *(volatile uint8_t*)address;
		if (value != START_BYTE)
		{
			currentWriteAddress = value;
			return;
		}
	}
}

void STMFlash::programByte(uint8_t data)
{
	if (checkAddress(currentWriteAddress))
	{
		HAL_StatusTypeDef resultProgram = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, currentWriteAddress, data);
		currentWriteAddress += 0x01; // size of byte
	}
}

void STMFlash::programHalfWord(uint16_t data)
{

}

void STMFlash::programWord(uint32_t data)
{

}

void STMFlash::programDoubleWord(uint64_t data)
{

}






