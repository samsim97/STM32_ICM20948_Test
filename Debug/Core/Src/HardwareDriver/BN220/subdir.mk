################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/HardwareDriver/BN220/BN220.cpp \
../Core/Src/HardwareDriver/BN220/GGAMessage.cpp \
../Core/Src/HardwareDriver/BN220/GLLMessage.cpp \
../Core/Src/HardwareDriver/BN220/GPSDataDeserializer.cpp \
../Core/Src/HardwareDriver/BN220/GPSParser.cpp \
../Core/Src/HardwareDriver/BN220/NMEAMessage.cpp \
../Core/Src/HardwareDriver/BN220/NMEAMessageFactory.cpp \
../Core/Src/HardwareDriver/BN220/String.cpp 

OBJS += \
./Core/Src/HardwareDriver/BN220/BN220.o \
./Core/Src/HardwareDriver/BN220/GGAMessage.o \
./Core/Src/HardwareDriver/BN220/GLLMessage.o \
./Core/Src/HardwareDriver/BN220/GPSDataDeserializer.o \
./Core/Src/HardwareDriver/BN220/GPSParser.o \
./Core/Src/HardwareDriver/BN220/NMEAMessage.o \
./Core/Src/HardwareDriver/BN220/NMEAMessageFactory.o \
./Core/Src/HardwareDriver/BN220/String.o 

CPP_DEPS += \
./Core/Src/HardwareDriver/BN220/BN220.d \
./Core/Src/HardwareDriver/BN220/GGAMessage.d \
./Core/Src/HardwareDriver/BN220/GLLMessage.d \
./Core/Src/HardwareDriver/BN220/GPSDataDeserializer.d \
./Core/Src/HardwareDriver/BN220/GPSParser.d \
./Core/Src/HardwareDriver/BN220/NMEAMessage.d \
./Core/Src/HardwareDriver/BN220/NMEAMessageFactory.d \
./Core/Src/HardwareDriver/BN220/String.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/HardwareDriver/BN220/%.o Core/Src/HardwareDriver/BN220/%.su Core/Src/HardwareDriver/BN220/%.cyclo: ../Core/Src/HardwareDriver/BN220/%.cpp Core/Src/HardwareDriver/BN220/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-HardwareDriver-2f-BN220

clean-Core-2f-Src-2f-HardwareDriver-2f-BN220:
	-$(RM) ./Core/Src/HardwareDriver/BN220/BN220.cyclo ./Core/Src/HardwareDriver/BN220/BN220.d ./Core/Src/HardwareDriver/BN220/BN220.o ./Core/Src/HardwareDriver/BN220/BN220.su ./Core/Src/HardwareDriver/BN220/GGAMessage.cyclo ./Core/Src/HardwareDriver/BN220/GGAMessage.d ./Core/Src/HardwareDriver/BN220/GGAMessage.o ./Core/Src/HardwareDriver/BN220/GGAMessage.su ./Core/Src/HardwareDriver/BN220/GLLMessage.cyclo ./Core/Src/HardwareDriver/BN220/GLLMessage.d ./Core/Src/HardwareDriver/BN220/GLLMessage.o ./Core/Src/HardwareDriver/BN220/GLLMessage.su ./Core/Src/HardwareDriver/BN220/GPSDataDeserializer.cyclo ./Core/Src/HardwareDriver/BN220/GPSDataDeserializer.d ./Core/Src/HardwareDriver/BN220/GPSDataDeserializer.o ./Core/Src/HardwareDriver/BN220/GPSDataDeserializer.su ./Core/Src/HardwareDriver/BN220/GPSParser.cyclo ./Core/Src/HardwareDriver/BN220/GPSParser.d ./Core/Src/HardwareDriver/BN220/GPSParser.o ./Core/Src/HardwareDriver/BN220/GPSParser.su ./Core/Src/HardwareDriver/BN220/NMEAMessage.cyclo ./Core/Src/HardwareDriver/BN220/NMEAMessage.d ./Core/Src/HardwareDriver/BN220/NMEAMessage.o ./Core/Src/HardwareDriver/BN220/NMEAMessage.su ./Core/Src/HardwareDriver/BN220/NMEAMessageFactory.cyclo ./Core/Src/HardwareDriver/BN220/NMEAMessageFactory.d ./Core/Src/HardwareDriver/BN220/NMEAMessageFactory.o ./Core/Src/HardwareDriver/BN220/NMEAMessageFactory.su ./Core/Src/HardwareDriver/BN220/String.cyclo ./Core/Src/HardwareDriver/BN220/String.d ./Core/Src/HardwareDriver/BN220/String.o ./Core/Src/HardwareDriver/BN220/String.su

.PHONY: clean-Core-2f-Src-2f-HardwareDriver-2f-BN220

