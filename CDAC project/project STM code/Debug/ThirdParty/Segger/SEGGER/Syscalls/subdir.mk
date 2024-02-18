################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c \
../ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.c \
../ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.c \
../ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.c 

OBJS += \
./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o \
./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o \
./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o \
./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o 

C_DEPS += \
./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d \
./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d \
./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d \
./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/Segger/SEGGER/Syscalls/%.o ThirdParty/Segger/SEGGER/Syscalls/%.su ThirdParty/Segger/SEGGER/Syscalls/%.cyclo: ../ThirdParty/Segger/SEGGER/Syscalls/%.c ThirdParty/Segger/SEGGER/Syscalls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"D:/STM32/Project_CDAC1/ThirdParty" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/include" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/portable" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/portable/GCC" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/portable/GCC/ARM_CM4F" -I"D:/STM32/Project_CDAC1/ThirdParty/FreeRTOS/portable/MemMang" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger/Config" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger/OS" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger/SEGGER" -I"D:/STM32/Project_CDAC1/ThirdParty/Segger/SEGGER/Syscalls" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-Segger-2f-SEGGER-2f-Syscalls

clean-ThirdParty-2f-Segger-2f-SEGGER-2f-Syscalls:
	-$(RM) ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.cyclo ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.su ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.cyclo ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.su ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.cyclo ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.su ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.cyclo ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o ./ThirdParty/Segger/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.su

.PHONY: clean-ThirdParty-2f-Segger-2f-SEGGER-2f-Syscalls

