/*
 * ICM20948.c
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

// *** Three asterisks to the side of a line means this may change based on platform
#include "main.h" // ***
// #include "stm32f4xx_hal_gpio.h" // ***
//#include "stm32f4xx_hal_i2c.h"  // ***
//#include "usart.h"// ***
//#include "stm32f4xx_hal_dma.h"  // ***
#include "ICM20948.h"
//#include <string.h>


#define DEV_ADDRESS 0x69 << 1
#define SENSORS_GRAVITY_EARTH (9.80665F)

I2C_HandleTypeDef* i2cHandle;

uint16_t accel_data[3];
uint16_t gyro_data[3];
int16_t mag_data[3];
/*
 *
 * SPI abstraction
 *
 */
void ICM_Constructor(I2C_HandleTypeDef* i2chandle)
{
	i2cHandle = i2chandle;
}

void ICM_readBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	//reg = reg | 0x80;
	//HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef testTx = HAL_I2C_Master_Transmit(I2C_BUS, DEV_ADDRESS, &reg, 1, HAL_TIMEOUT);
	HAL_StatusTypeDef testRx = HAL_I2C_Master_Receive(I2C_BUS, DEV_ADDRESS, pData, Size, HAL_TIMEOUT);
	//HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
	//HAL_SPI_Receive_DMA(SPI_BUS, pData, Size);
	//HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

void ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	//reg = reg & 0x7F;
	//HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
	HAL_I2C_Master_Transmit(I2C_BUS, DEV_ADDRESS, &reg, 1, HAL_TIMEOUT);
	HAL_I2C_Master_Transmit(I2C_BUS, DEV_ADDRESS, pData, Size, HAL_TIMEOUT);
	//HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
	//HAL_SPI_Transmit_DMA(SPI_BUS, pData, Size);
	//HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);

}

void ICM_ReadOneByte(uint8_t reg, uint8_t* pData) // ***
{
	//reg = reg | 0x80;
	//HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef testTx = HAL_I2C_Master_Transmit(I2C_BUS, DEV_ADDRESS, &reg, 1, HAL_TIMEOUT);
	/*while (HAL_I2C_GetState(I2C_BUS) != HAL_I2C_STATE_READY)
		;*/
	HAL_StatusTypeDef testRx = HAL_I2C_Master_Receive(I2C_BUS, DEV_ADDRESS, pData, 1, HAL_TIMEOUT);
	/*while (HAL_I2C_GetState(I2C_BUS) != HAL_I2C_STATE_READY)
			;*/
	//HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
	/*while (HAL_SPI_GetState(SPI_BUS) != HAL_I2C_STATE_READY)
		;
	HAL_SPI_Receive_DMA(SPI_BUS, pData, 1);
	while (HAL_SPI_GetState(SPI_BUS) != HAL_I2C_STATE_READY)
		;
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);*/
}

void ICM_WriteOneByte(uint8_t reg, uint8_t Data) // ***
{
	//reg = reg & 0x7F;
	//HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
	/*HAL_StatusTypeDef testTx = HAL_BUSY;
	while (testTx != HAL_OK)
	{
		testTx = HAL_I2C_Master_Transmit(I2C_BUS, DEV_ADDRESS, &reg, 1, HAL_TIMEOUT);
		HAL_Delay(50);
	}*/
	uint8_t data[2] = {reg, Data};
	HAL_StatusTypeDef testTx = HAL_I2C_Master_Transmit(I2C_BUS, DEV_ADDRESS, &data, 2, HAL_TIMEOUT);
	//HAL_StatusTypeDef testTx2 = HAL_I2C_Master_Transmit(I2C_BUS, DEV_ADDRESS, &Data, 1, HAL_TIMEOUT);
	//HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

/*
 *
 * AUX I2C abstraction for magnetometer
 *
 */
void i2c_Mag_write(uint8_t reg,uint8_t value)
  {
  	ICM_WriteOneByte(0x7F, 0x30);

  	HAL_Delay(1);
  	ICM_WriteOneByte(0x03 ,0x0C);//mode: write

  	HAL_Delay(1);
  	ICM_WriteOneByte(0x04 ,reg);//set reg addr

  	HAL_Delay(1);
  	ICM_WriteOneByte(0x06 ,value);//send value

  	HAL_Delay(1);
  }

  static uint8_t ICM_Mag_Read(uint8_t reg)
  {
  	uint8_t  Data;
  	ICM_WriteOneByte(0x7F, 0x30);
    HAL_Delay(1);
  	ICM_WriteOneByte(0x03 ,0x0C|0x80);
    HAL_Delay(1);
  	ICM_WriteOneByte(0x04 ,reg);// set reg addr
    HAL_Delay(1);
  	ICM_WriteOneByte(0x06 ,0xff);//read
  	HAL_Delay(1);
  	ICM_WriteOneByte(0x7F, 0x00);
  	ICM_ReadOneByte(0x3B,&Data);
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

/*
 *
 * Read magnetometer
 *
 */
void ICM_ReadMag(int16_t magn[3]) {
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

/*
 *
 * Sequence to setup ICM290948 as early as possible after power on
 *
 */
void ICM_PowerOn(void) {
		//ICM_CSHigh();
		//HAL_Delay(10);
		ICM_SelectBank(USER_BANK_0);
		//HAL_Delay(10);
		//ICM_Disable_I2C();
		HAL_Delay(10);
		ICM_SetClock((uint8_t)CLK_BEST_AVAIL);
		HAL_Delay(10);
		ICM_AccelGyroOff();
		HAL_Delay(20);
		ICM_AccelGyroOn();
		HAL_Delay(10);
		ICM_Initialize();
		//uint8_t test = ICM_WHOAMI();
	//} else {
		//sprintf(uart_buffer, "Failed WHO_AM_I.  %i is not 0xEA\r\n", test);
		//HAL_UART_Transmit_DMA(UART_BUS, (uint8_t*) uart_buffer, strlen(uart_buffer));
		//HAL_Delay(100);
	//}
}
uint16_t ICM_Initialize(void) {
		ICM_SelectBank(USER_BANK_2);
		HAL_Delay(20);
		ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
		HAL_Delay(10);

		// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
		ICM_WriteOneByte(0x00, 0x0A);
		HAL_Delay(10);

		// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
		ICM_WriteOneByte(0x14, (0x04 | 0x11));

		// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
		ICM_WriteOneByte(0x10, 0x00);
		HAL_Delay(10);

		// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
		ICM_WriteOneByte(0x11, 0x0A);
		HAL_Delay(10);

		ICM_SelectBank(USER_BANK_2);
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
		ICM_SelectBank(USER_BANK_0);
		HAL_Delay(20);
		uint8_t testSleep = 0x00;
		uint8_t testSleep2 = 0x00;
		uint8_t testconfig = 0x00;
		ICM_ReadOneByte(0x06, &testSleep);
		HAL_Delay(10);
		ICM_ReadOneByte(0x03, &testconfig);
		HAL_Delay(10);
		// Remove sleep
		ICM_SelectBank(USER_BANK_0);
		HAL_Delay(20);
		ICM_WriteOneByte(0x06, 0x01);
		HAL_Delay(10);
		ICM_ReadOneByte(0x06, &testSleep2);
		HAL_Delay(10);
		ICM_SelectBank(USER_BANK_2);

		return 1337;
	}

void ICM_ReadAccelGyro(void) {
	ICM_SelectBank(USER_BANK_0);
	uint8_t whoAmI = 0;
	ICM_ReadOneByte(0x00,&whoAmI);
	uint8_t raw_data[12] = {0};
	ICM_readBytes(0x2D, raw_data, 12);

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

	float ax = accel_data[0] / 4096.0;
	float ay = accel_data[1] / 4096.0;
	float az = accel_data[2] / 4096.0;

	float ax2 = ax * SENSORS_GRAVITY_EARTH;
	float ay2 = ay * SENSORS_GRAVITY_EARTH;
	float az2 = az * SENSORS_GRAVITY_EARTH;


	gyro_data[0] = gyro_data[0] / 250;
	gyro_data[1] = gyro_data[1] / 250;
	gyro_data[2] = gyro_data[2] / 250;
}
void ICM_SelectBank(uint8_t bank) {
	ICM_WriteOneByte(USER_BANK_SEL, bank);
}
void ICM_Disable_I2C(void) {
	ICM_WriteOneByte(0x03, 0x78);
}
/*void ICM_CSHigh(void) {
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, SET);
}
void ICM_CSLow(void) {
	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, RESET);
}*/
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
}


/*
 *
 * Read Accelerometer and Gyro data
 *
 */
