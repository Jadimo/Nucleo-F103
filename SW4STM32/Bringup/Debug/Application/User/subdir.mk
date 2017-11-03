################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
F:/STM/Code/Bringup/Src/main.c \
F:/STM/Code/Bringup/Src/stm32f1xx_hal_msp.c \
F:/STM/Code/Bringup/Src/stm32f1xx_it.c 

OBJS += \
./Application/User/main.o \
./Application/User/stm32f1xx_hal_msp.o \
./Application/User/stm32f1xx_it.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/stm32f1xx_hal_msp.d \
./Application/User/stm32f1xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: F:/STM/Code/Bringup/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"F:/STM/Code/Bringup/Inc" -I"F:/STM/Code/Bringup/Drivers/STM32F1xx_HAL_Driver/Inc" -I"F:/STM/Code/Bringup/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"F:/STM/Code/Bringup/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"F:/STM/Code/Bringup/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f1xx_hal_msp.o: F:/STM/Code/Bringup/Src/stm32f1xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"F:/STM/Code/Bringup/Inc" -I"F:/STM/Code/Bringup/Drivers/STM32F1xx_HAL_Driver/Inc" -I"F:/STM/Code/Bringup/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"F:/STM/Code/Bringup/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"F:/STM/Code/Bringup/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f1xx_it.o: F:/STM/Code/Bringup/Src/stm32f1xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"F:/STM/Code/Bringup/Inc" -I"F:/STM/Code/Bringup/Drivers/STM32F1xx_HAL_Driver/Inc" -I"F:/STM/Code/Bringup/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"F:/STM/Code/Bringup/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"F:/STM/Code/Bringup/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


