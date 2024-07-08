/*
 * ICM20948.c
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

// *** Three asterisks to the side of a line means this may change based on platform
#include <HardwareDriver/ICM20948/ICM20948.hpp>

// #include "stm32f4xx_hal_gpio.h" // ***
//#include "stm32f4xx_hal_i2c.h"  // ***
//#include "usart.h"// ***
//#include "stm32f4xx_hal_dma.h"  // ***
//#include <string.h>


#define DEV_ADDRESS 0x69 << 1
#define SENSORS_GRAVITY_EARTH (9.80665F)

ICM20948::ICM20948(I2C_HandleTypeDef* i2cHandle)
{
	this->i2cHandle = i2cHandle;
	//accelerometerScaleDivider = ACCELEROMETER_DEFAULT_SCALE_DIVIDER;
	accelerometerScaleDivider = ACCELEROMETER_8G_SCALE_DIVIDER;
}

/*************************************/
/***       BOARD MANAGEMENT        ***/
/*************************************/

uint16_t ICM20948::init()
{
	// POWER ON
	selectUserBank(USER_BANK_0);
	HAL_Delay(10);
	setClock((uint8_t)CLK_BEST_AVAIL);
	HAL_Delay(10);
	writeRegister(PWR_MGMT_2, (0x38 | 0x07)); // TURN ACCEL AND GYRO OFF
	HAL_Delay(20);
	writeRegister(0x07, (0x00 | 0x00)); // TURN ACCEL AND GYRO ON
	HAL_Delay(10);

	// INIT
	selectUserBank(USER_BANK_2);
	HAL_Delay(20);
	//ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
	writeRegister(GYRO_CONFIG_1, (GYRO_RATE_250|GYRO_LPF_17HZ));
	HAL_Delay(10);

	// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
	writeRegister(0x00, 0x0A);
	HAL_Delay(10);

	// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
	//ICM_WriteOneByte(0x14, (0x04 | 0x11));
	writeRegister(0x14, 0x04);
	HAL_Delay(10);

	// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
	writeRegister(0x10, 0x00);
	HAL_Delay(10);

	// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
	writeRegister(0x11, 0x0A);
	HAL_Delay(10);

	selectUserBank(USER_BANK_2);
	HAL_Delay(20);

	// Configure AUX_I2C Magnetometer (onboard ICM-20948)
	//ICM_WriteOneByte(0x7F, 0x00); // Select user bank 0
	//ICM_WriteOneByte(0x0F, 0x30); // INT Pin / Bypass Enable Configuration
	//ICM_WriteOneByte(0x03, 0x20); // I2C_MST_EN
	//ICM_WriteOneByte(0x7F, 0x30); // Select user bank 3
	//ICM_WriteOneByte(0x01, 0x4D); // I2C Master mode and Speed 400 kHz
	//ICM_WriteOneByte(0x02, 0x01); // I2C_SLV0 _DLY_ enable
	//ICM_WriteOneByte(0x05, 0x81); // enable IIC	and EXT_SENS_DATA==1 Byte

	// Initialize magnetometer
	//i2c_Mag_write(0x32, 0x01); // Reset AK8963
	//HAL_Delay(1000);
	//i2c_Mag_write(0x31, 0x02); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output
	selectUserBank(USER_BANK_0);
	HAL_Delay(20);
	/*uint8_t testSleep = 0x00;
	uint8_t testSleep2 = 0x00;
	uint8_t testconfig = 0x00;
	ICM_ReadOneByte(0x06, &testSleep);
	HAL_Delay(10);
	ICM_ReadOneByte(0x03, &testconfig);
	HAL_Delay(10);*/
	// Remove sleep
	selectUserBank(USER_BANK_0);
	HAL_Delay(20);
	writeRegister(0x06, 0x01);
	HAL_Delay(10);
	/*ICM_ReadOneByte(0x06, &testSleep2);
	HAL_Delay(10);*/
	selectUserBank(USER_BANK_2);

	return 1337;
}

bool ICM20948::checkCommunication()
{
	uint8_t value = 0x01;
	readRegister(0x00, &value);
	return true; // CHECK IF VALUE IS GOOD
}

void ICM20948::selectUserBank(uint8_t bankNumber)
{
	writeRegister(USER_BANK_SEL, bankNumber);
}

void ICM20948::setClock(uint8_t rate)
{
	writeRegister(PWR_MGMT_1, rate);
}

/*************************************/
/***         Accelerometer         ***/
/*************************************/

void ICM20948::readAccelerometer()
{
	const int BUFFER_SIZE = 6; // 3 uint16_t stored in 6 registers
	uint8_t raw_data[BUFFER_SIZE] = {0};

	selectUserBank(USER_BANK_0);
	HAL_Delay(20);

	readRegisters(0x2D, raw_data, BUFFER_SIZE);

	accelerometerValues.x_g = static_cast<int16_t>((raw_data[0] << 8) | raw_data[1]) / accelerometerScaleDivider;
	accelerometerValues.y_g = static_cast<int16_t>((raw_data[2] << 8) | raw_data[3]) / accelerometerScaleDivider;
	accelerometerValues.z_g = static_cast<int16_t>((raw_data[4] << 8) | raw_data[5]) / accelerometerScaleDivider;
}

AccelerometerValues ICM20948::getAccelerometerValues()
{
	return accelerometerValues;
}

/*************************************/
/***           Gyroscope           ***/
/*************************************/

void ICM20948::readGyroscope()
{
	const int BUFFER_SIZE = 6; // 3 uint16_t stored in 6 registers
	uint8_t raw_data[BUFFER_SIZE] = {0};

	selectUserBank(USER_BANK_0);
	HAL_Delay(20);

	readRegisters(0x33, raw_data, BUFFER_SIZE);

	gyroscopeValues.x_degPerSec = ((raw_data[6] << 8) | raw_data[7]) / GYROSCOPE_250DPS_SCALE_DIVIDER;
	gyroscopeValues.y_degPerSec = ((raw_data[8] << 8) | raw_data[9]) / GYROSCOPE_250DPS_SCALE_DIVIDER;
	gyroscopeValues.z_degPerSec = ((raw_data[10] << 8) | raw_data[11]) / GYROSCOPE_250DPS_SCALE_DIVIDER;
}

GyroscopeValues ICM20948::getGyroscopeValues()
{
	return gyroscopeValues;
}

/*************************************/
/***          Magnetometer         ***/
/*************************************/

void ICM20948::readMagnetometer()
{

}

MagnetometerValues ICM20948::getMagnetometerValues()
{
	return magnetometerValues;
}

/*************************************/
/***          Thermometer          ***/
/*************************************/

void ICM20948::readThermometer()
{

}

float ICM20948::getThermometerValue()
{
	return thermometerValue;
}

/*************************************/
/***         Communication         ***/
/*************************************/

void ICM20948::writeRegister(uint8_t registerAddress, uint8_t value)
{
	uint8_t data[2] = {registerAddress, value};
	HAL_I2C_Master_Transmit(i2cHandle, I2C_ADDRESS, data, 2, HAL_TIMEOUT);
}

void ICM20948::readRegister(uint8_t registerAddress, uint8_t* value)
{
	HAL_I2C_Master_Transmit(i2cHandle, I2C_ADDRESS, &registerAddress, 1, HAL_TIMEOUT);
	HAL_I2C_Master_Receive(i2cHandle, I2C_ADDRESS, value, 1, HAL_TIMEOUT);
}

void ICM20948::readRegisters(uint8_t registerAddress, uint8_t* dataBuffer, uint8_t bufferSize)
{
	HAL_I2C_Master_Transmit(i2cHandle, I2C_ADDRESS, &registerAddress, 1, HAL_TIMEOUT);
	HAL_I2C_Master_Receive(i2cHandle, I2C_ADDRESS, dataBuffer, bufferSize, HAL_TIMEOUT);
}

/*
 *
 * AUX I2C abstraction for magnetometer
 *
 */
/*void i2c_Mag_write(uint8_t reg,uint8_t value)
  {
	writeRegister(0x7F, 0x30);

  	HAL_Delay(1);
  	writeRegister(0x03 ,0x0C);//mode: write

  	HAL_Delay(1);
  	writeRegister(0x04 ,reg);//set reg addr

  	HAL_Delay(1);
  	writeRegister(0x06 ,value);//send value

  	HAL_Delay(1);
  }

  static uint8_t ICM_Mag_Read(uint8_t reg)
  {
  	uint8_t  Data;
  	writeRegister(0x7F, 0x30);
    HAL_Delay(1);
    writeRegister(0x03 ,0x0C|0x80);
    HAL_Delay(1);
    writeRegister(0x04 ,reg);// set reg addr
    HAL_Delay(1);
    writeRegister(0x06 ,0xff);//read
  	HAL_Delay(1);
  	writeRegister(0x7F, 0x00);
  	readRegister(0x3B,&Data);
    HAL_Delay(1);
  	return Data;
  }

  void ICM20948_READ_MAG(int16_t magn[3])
  {
    uint8_t mag_buffer[10];

      mag_buffer[0] =ICM_Mag_Read(0x01);

      mag_buffer[1] =ICM_Mag_Read(0x11);
  	  mag_buffer[2] =ICM_Mag_Read(0x12);
  	  magn[0]=mag_buffer[1]|mag_buffer[2]<<8;
    	mag_buffer[3] =ICM_Mag_Read(0x13);
      mag_buffer[4] =ICM_Mag_Read(0x14);
    	magn[1]=mag_buffer[3]|mag_buffer[4]<<8;
  	 	mag_buffer[5] =ICM_Mag_Read(0x15);
      mag_buffer[6] =ICM_Mag_Read(0x16);
  		magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

     	i2c_Mag_write(0x31,0x01);
  }


void ICM_ReadMag(int16_t magn[3]) {
	uint8_t mag_buffer[10];

	      mag_buffer[0] = ICM_Mag_Read(0x01);

	      mag_buffer[1] = ICM_Mag_Read(0x11);
	  	  mag_buffer[2] = ICM_Mag_Read(0x12);
	  	  magn[0] = mag_buffer[1]|mag_buffer[2]<<8;
	    	mag_buffer[3] = ICM_Mag_Read(0x13);
	      mag_buffer[4] = ICM_Mag_Read(0x14);
	    	magn[1]=mag_buffer[3]|mag_buffer[4]<<8;
	  	 	mag_buffer[5] =ICM_Mag_Read(0x15);
	      mag_buffer[6] =ICM_Mag_Read(0x16);
	  		magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

	     	i2c_Mag_write(0x31,0x01);
}

void ICM_ReadAccelGyro(void) {
	ICM_SelectBank(USER_BANK_0);
	HAL_Delay(20);
	//uint8_t whoAmI = 0;
	//ICM_ReadOneByte(0x00,&whoAmI);
	uint8_t raw_data[12] = {0};
	ICM_readBytes(0x2D, raw_data, 12); // 0x2D est le registre

	accel_data[0] = (raw_data[0] << 8) | raw_data[1];
	accel_data[1] = (raw_data[2] << 8) | raw_data[3];
	accel_data[2] = (raw_data[4] << 8) | raw_data[5];

	gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
	gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
	gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

	//accel_data[0] = accel_data[0] / 8;
	//accel_data[1] = accel_data[1] / 8;
	//accel_data[2] = accel_data[2] / 8;

	//accel_data[0] = accel_data[0] / 4096;
	//accel_data[1] = accel_data[1] / 4096;
	//accel_data[2] = accel_data[2] / 4096;

	float ax = accel_data[0] / 4096;
	float ay = accel_data[1] / 4096;
	float az = accel_data[2] / 4096;

	float ax2 = ax * SENSORS_GRAVITY_EARTH;
	float ay2 = ay * SENSORS_GRAVITY_EARTH;
	float az2 = az * SENSORS_GRAVITY_EARTH;


 	gyro_data[0] = gyro_data[0] / 250;
	gyro_data[1] = gyro_data[1] / 250;
	gyro_data[2] = gyro_data[2] / 250;
}

void ICM_Disable_I2C(void) {
	ICM_WriteOneByte(0x03, 0x78);
}

void ICM_SetClock(uint8_t clk) {
	ICM_WriteOneByte(PWR_MGMT_1, clk);
}
void ICM_AccelGyroOff(void) {
	ICM_WriteOneByte(PWR_MGMT_2, (0x38 | 0x07));
}

void ICM_AccelGyroOn(void) {
	ICM_WriteOneByte(0x07, (0x00 | 0x00));
}

uint8_t ICM_WHOAMI(void) {
	uint8_t spiData = 0x01;
	ICM_ReadOneByte(0x00, &spiData);
	return spiData;
}
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf) {
	ICM_WriteOneByte(GYRO_CONFIG_1, (rate|lpf));
}

uint8_t ICM_GetAccelRange(void)
{
	uint8_t rawData = 0x00;
	ICM_SelectBank(USER_BANK_2);
	ICM_ReadOneByte(0x14, &rawData);
	ICM_SelectBank(USER_BANK_0);
	return rawData;
}*/
