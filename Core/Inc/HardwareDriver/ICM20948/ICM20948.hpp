#pragma once

#include <HardwareDriver/ICM20948/ICM20948Defines.hpp>
#include <HardwareDriver/ICM20948/ICM20948Registers.hpp>
#include <stdint.h>
#include "stm32f4xx_hal.h"

#include <Sensors/Accelerometer/IAccelerometerDriver.hpp>
#include <Sensors/Accelerometer/AccelerometerValues.hpp>

#include <Sensors/Gyroscope/IGyroscopeDriver.hpp>
#include <Sensors/Gyroscope/GyroscopeValues.hpp>
#include <Sensors/Magnetometer/MagnetometerValues.hpp>

#define PWR_MGMT_1 		(0x06)
#define PWR_MGMT_2		(0x07)
#define GYRO_CONFIG_1	(0x01)


#define CLK_BEST_AVAIL	(0x01)
#define GYRO_RATE_250	(0x00)
#define GYRO_LPF_17HZ 	(0x29)

class ICM20948 : public IAccelerometerDriver, public IGyroscopeDriver
{
public:
	ICM20948(I2C_HandleTypeDef* i2chandle);
	~ICM20948();

	// Board management
	uint16_t init();
	bool checkCommunication();
	void selectUserBank(uint8_t bankNumber);
	void setClock(uint8_t rate);


	// Accelerometer
	void readAccelerometer();
	AccelerometerValues getAccelerometerValues();

	// Gyroscope
	void readGyroscope();
	GyroscopeValues getGyroscopeValues();

	// Magnetometer
	void readMagnetometer();
	MagnetometerValues getMagnetometerValues();

	// Thermometer
	void readThermometer();
	float getThermometerValue();

	// Communication
	void writeRegister(uint8_t registerAddress, uint8_t value);
	void readRegister(uint8_t registerAddress, uint8_t* value);
	void readRegisters(uint8_t registerAddress, uint8_t* buffer, uint8_t bufferSize);

private:
	I2C_HandleTypeDef* i2cHandle;

	AccelerometerValues accelerometerValues;
	float accelerometerScaleDivider;

	GyroscopeValues gyroscopeValues;
	float gyroscopeScaleDivider;

	MagnetometerValues magnetometerValues;
	float thermometerValue;
};

/*void ICM_ReadAccelGyro(void);
void ICM_ReadMag(int16_t magn[3]);
uint16_t ICM_Initialize(void);
void ICM_SelectBank(uint8_t bank);
void ICM_SetClock(uint8_t clk);
void ICM_AccelGyroOff(void);
void ICM_AccelGyroOn(void);
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf);
void ICM_SetGyroLPF(uint8_t lpf);
uint8_t ICM_GetAccelRange(void);*/
