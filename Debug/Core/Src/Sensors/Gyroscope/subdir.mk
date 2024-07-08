################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Sensors/Gyroscope/Gyroscope.cpp 

OBJS += \
./Core/Src/Sensors/Gyroscope/Gyroscope.o 

CPP_DEPS += \
./Core/Src/Sensors/Gyroscope/Gyroscope.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Sensors/Gyroscope/%.o Core/Src/Sensors/Gyroscope/%.su Core/Src/Sensors/Gyroscope/%.cyclo: ../Core/Src/Sensors/Gyroscope/%.cpp Core/Src/Sensors/Gyroscope/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Sensors-2f-Gyroscope

clean-Core-2f-Src-2f-Sensors-2f-Gyroscope:
	-$(RM) ./Core/Src/Sensors/Gyroscope/Gyroscope.cyclo ./Core/Src/Sensors/Gyroscope/Gyroscope.d ./Core/Src/Sensors/Gyroscope/Gyroscope.o ./Core/Src/Sensors/Gyroscope/Gyroscope.su

.PHONY: clean-Core-2f-Src-2f-Sensors-2f-Gyroscope

