################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Rocket/Rocket.cpp 

OBJS += \
./Core/Src/Rocket/Rocket.o 

CPP_DEPS += \
./Core/Src/Rocket/Rocket.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Rocket/%.o Core/Src/Rocket/%.su Core/Src/Rocket/%.cyclo: ../Core/Src/Rocket/%.cpp Core/Src/Rocket/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Rocket

clean-Core-2f-Src-2f-Rocket:
	-$(RM) ./Core/Src/Rocket/Rocket.cyclo ./Core/Src/Rocket/Rocket.d ./Core/Src/Rocket/Rocket.o ./Core/Src/Rocket/Rocket.su

.PHONY: clean-Core-2f-Src-2f-Rocket

