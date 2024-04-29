################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/ICM20948/Adafruit_BusIO_Register.cpp \
../Core/Src/ICM20948/Adafruit_I2CDevice.cpp \
../Core/Src/ICM20948/Adafruit_ICM20948.cpp \
../Core/Src/ICM20948/Adafruit_ICM20X.cpp \
../Core/Src/ICM20948/Adafruit_Sensor.cpp 

OBJS += \
./Core/Src/ICM20948/Adafruit_BusIO_Register.o \
./Core/Src/ICM20948/Adafruit_I2CDevice.o \
./Core/Src/ICM20948/Adafruit_ICM20948.o \
./Core/Src/ICM20948/Adafruit_ICM20X.o \
./Core/Src/ICM20948/Adafruit_Sensor.o 

CPP_DEPS += \
./Core/Src/ICM20948/Adafruit_BusIO_Register.d \
./Core/Src/ICM20948/Adafruit_I2CDevice.d \
./Core/Src/ICM20948/Adafruit_ICM20948.d \
./Core/Src/ICM20948/Adafruit_ICM20X.d \
./Core/Src/ICM20948/Adafruit_Sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ICM20948/%.o Core/Src/ICM20948/%.su Core/Src/ICM20948/%.cyclo: ../Core/Src/ICM20948/%.cpp Core/Src/ICM20948/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ICM20948

clean-Core-2f-Src-2f-ICM20948:
	-$(RM) ./Core/Src/ICM20948/Adafruit_BusIO_Register.cyclo ./Core/Src/ICM20948/Adafruit_BusIO_Register.d ./Core/Src/ICM20948/Adafruit_BusIO_Register.o ./Core/Src/ICM20948/Adafruit_BusIO_Register.su ./Core/Src/ICM20948/Adafruit_I2CDevice.cyclo ./Core/Src/ICM20948/Adafruit_I2CDevice.d ./Core/Src/ICM20948/Adafruit_I2CDevice.o ./Core/Src/ICM20948/Adafruit_I2CDevice.su ./Core/Src/ICM20948/Adafruit_ICM20948.cyclo ./Core/Src/ICM20948/Adafruit_ICM20948.d ./Core/Src/ICM20948/Adafruit_ICM20948.o ./Core/Src/ICM20948/Adafruit_ICM20948.su ./Core/Src/ICM20948/Adafruit_ICM20X.cyclo ./Core/Src/ICM20948/Adafruit_ICM20X.d ./Core/Src/ICM20948/Adafruit_ICM20X.o ./Core/Src/ICM20948/Adafruit_ICM20X.su ./Core/Src/ICM20948/Adafruit_Sensor.cyclo ./Core/Src/ICM20948/Adafruit_Sensor.d ./Core/Src/ICM20948/Adafruit_Sensor.o ./Core/Src/ICM20948/Adafruit_Sensor.su

.PHONY: clean-Core-2f-Src-2f-ICM20948

