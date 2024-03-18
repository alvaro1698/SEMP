################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/l3gd20/l3gd20.c 

OBJS += \
./Components/l3gd20/l3gd20.o 

C_DEPS += \
./Components/l3gd20/l3gd20.d 


# Each subdirectory must supply rules for building sources it contributes
Components/l3gd20/%.o Components/l3gd20/%.su Components/l3gd20/%.cyclo: ../Components/l3gd20/%.c Components/l3gd20/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -DARM_MATH_CM4=1 -c -I../Core/Inc -I../Middlewares/ST/ARM/DSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-l3gd20

clean-Components-2f-l3gd20:
	-$(RM) ./Components/l3gd20/l3gd20.cyclo ./Components/l3gd20/l3gd20.d ./Components/l3gd20/l3gd20.o ./Components/l3gd20/l3gd20.su

.PHONY: clean-Components-2f-l3gd20

