################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Sensors/ICM20948/ICM20948.cpp 

OBJS += \
./Core/Src/Sensors/ICM20948/ICM20948.o 

CPP_DEPS += \
./Core/Src/Sensors/ICM20948/ICM20948.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Sensors/ICM20948/%.o Core/Src/Sensors/ICM20948/%.su Core/Src/Sensors/ICM20948/%.cyclo: ../Core/Src/Sensors/ICM20948/%.cpp Core/Src/Sensors/ICM20948/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Sensors-2f-ICM20948

clean-Core-2f-Src-2f-Sensors-2f-ICM20948:
	-$(RM) ./Core/Src/Sensors/ICM20948/ICM20948.cyclo ./Core/Src/Sensors/ICM20948/ICM20948.d ./Core/Src/Sensors/ICM20948/ICM20948.o ./Core/Src/Sensors/ICM20948/ICM20948.su

.PHONY: clean-Core-2f-Src-2f-Sensors-2f-ICM20948

