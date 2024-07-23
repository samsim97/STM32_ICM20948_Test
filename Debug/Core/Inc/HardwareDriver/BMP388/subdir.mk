################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/HardwareDriver/BMP388/bmp3.c 

C_DEPS += \
./Core/Inc/HardwareDriver/BMP388/bmp3.d 

OBJS += \
./Core/Inc/HardwareDriver/BMP388/bmp3.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/HardwareDriver/BMP388/%.o Core/Inc/HardwareDriver/BMP388/%.su Core/Inc/HardwareDriver/BMP388/%.cyclo: ../Core/Inc/HardwareDriver/BMP388/%.c Core/Inc/HardwareDriver/BMP388/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-HardwareDriver-2f-BMP388

clean-Core-2f-Inc-2f-HardwareDriver-2f-BMP388:
	-$(RM) ./Core/Inc/HardwareDriver/BMP388/bmp3.cyclo ./Core/Inc/HardwareDriver/BMP388/bmp3.d ./Core/Inc/HardwareDriver/BMP388/bmp3.o ./Core/Inc/HardwareDriver/BMP388/bmp3.su

.PHONY: clean-Core-2f-Inc-2f-HardwareDriver-2f-BMP388

