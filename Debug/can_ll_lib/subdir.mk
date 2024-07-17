################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../can_ll_lib/stm32f4_can_ll.c 

OBJS += \
./can_ll_lib/stm32f4_can_ll.o 

C_DEPS += \
./can_ll_lib/stm32f4_can_ll.d 


# Each subdirectory must supply rules for building sources it contributes
can_ll_lib/%.o can_ll_lib/%.su can_ll_lib/%.cyclo: ../can_ll_lib/%.c can_ll_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F405xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../CAN_LL -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I"F:/STM32project/STM32F103C8T6/CODE/STM32F405_CAN_LL_/can_ll_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-can_ll_lib

clean-can_ll_lib:
	-$(RM) ./can_ll_lib/stm32f4_can_ll.cyclo ./can_ll_lib/stm32f4_can_ll.d ./can_ll_lib/stm32f4_can_ll.o ./can_ll_lib/stm32f4_can_ll.su

.PHONY: clean-can_ll_lib

