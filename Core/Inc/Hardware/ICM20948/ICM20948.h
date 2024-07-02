#pragma once

#include <Hardware/ICM20948/ICM20948Registers.hpp>

#define UART_BUS		(&huart3) // ***
#define I2C_BUS 		(i2cHandle)

#define PWR_MGMT_1 		(0x06)
#define PWR_MGMT_2		(0x07)
#define GYRO_CONFIG_1	(0x01)


#define CLK_BEST_AVAIL	(0x01)
#define GYRO_RATE_250	(0x00)
#define GYRO_LPF_17HZ 	(0x29)

void ICM_Constructor(I2C_HandleTypeDef* i2chandle);
void ICM_PowerOn();
uint8_t ICM_WHOAMI(void);
void ICM_SelectBank(uint8_t bank);
void ICM_ReadAccelGyro(void);
void ICM_ReadMag(int16_t magn[3]);
uint16_t ICM_Initialize(void);
void ICM_SelectBank(uint8_t bank);
void ICM_Disable_I2C(void);
void ICM_CSHigh(void);
void ICM_CSLow(void);
void ICM_SetClock(uint8_t clk);
void ICM_AccelGyroOff(void);
void ICM_AccelGyroOn(void);
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf);
void ICM_SetGyroLPF(uint8_t lpf);
uint8_t ICM_GetAccelRange(void);
