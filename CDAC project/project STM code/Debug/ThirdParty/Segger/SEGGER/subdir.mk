################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/Segger/SEGGER/SEGGER_RTT.c \
../ThirdParty/Segger/SEGGER/SEGGER_RTT_printf.c \
../ThirdParty/Segger/SEGGER/SEGGER_SYSVIEW.c 

S_UPPER_SRCS += \
../ThirdParty/Segger/SEGGER/SEGGER_RTT_ASM_ARMv7M.S 

OBJS += \
./ThirdParty/Segger/SEGGER/SEGGER_RTT.o \
./ThirdParty/Segger/SEGGER/SEGGER_RTT_ASM_ARMv7M.o \
./ThirdParty/Segger/SEGGER/SEGGER_RTT_printf.o \
./ThirdParty/Segger/SEGGER/SEGGER_SYSVIEW.o 

S_UPPER_DEPS += \
./ThirdParty/Segger/SEGGER/SEGGER_RTT_ASM_ARMv7M.d 

C_DEPS += \
./ThirdParty/Segger/SEGGER/SEGGER_RTT.d \
./ThirdParty/Segger/SEGGER/SEGGER_RTT_printf.d \
./ThirdParty/Segger/SEGGER/SEGGER_SYSVIEW.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/Segger/SEGGER/%.o ThirdParty/Segger/SEGGER/%.su ThirdParty/Segger/SEGGER/%.cyclo: ../ThirdParty/Segger/SEGGER/%.c ThirdParty/Segger/SEGGER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"D:/STM32/Project_CDAC1/ThirdParty" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/include" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/portable" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/portable/GCC" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/portable/MemMang" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger/Config" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger/OS" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger/SEGGER" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger/SEGGER/Syscalls" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ThirdParty/Segger/SEGGER/%.o: ../ThirdParty/Segger/SEGGER/%.S ThirdParty/Segger/SEGGER/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"D:/STM32/Project_CDAC1/ThirdParty/Segger/Config" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-ThirdParty-2f-Segger-2f-SEGGER

clean-ThirdParty-2f-Segger-2f-SEGGER:
	-$(RM) ./ThirdParty/Segger/SEGGER/SEGGER_RTT.cyclo ./ThirdParty/Segger/SEGGER/SEGGER_RTT.d ./ThirdParty/Segger/SEGGER/SEGGER_RTT.o ./ThirdParty/Segger/SEGGER/SEGGER_RTT.su ./ThirdParty/Segger/SEGGER/SEGGER_RTT_ASM_ARMv7M.d ./ThirdParty/Segger/SEGGER/SEGGER_RTT_ASM_ARMv7M.o ./ThirdParty/Segger/SEGGER/SEGGER_RTT_printf.cyclo ./ThirdParty/Segger/SEGGER/SEGGER_RTT_printf.d ./ThirdParty/Segger/SEGGER/SEGGER_RTT_printf.o ./ThirdParty/Segger/SEGGER/SEGGER_RTT_printf.su ./ThirdParty/Segger/SEGGER/SEGGER_SYSVIEW.cyclo ./ThirdParty/Segger/SEGGER/SEGGER_SYSVIEW.d ./ThirdParty/Segger/SEGGER/SEGGER_SYSVIEW.o ./ThirdParty/Segger/SEGGER/SEGGER_SYSVIEW.su

.PHONY: clean-ThirdParty-2f-Segger-2f-SEGGER

