################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Components/otm8009a/otm8009a.c 

OBJS += \
./Components/otm8009a/otm8009a.o 

C_DEPS += \
./Components/otm8009a/otm8009a.d 


# Each subdirectory must supply rules for building sources it contributes
Components/otm8009a/%.o Components/otm8009a/%.su Components/otm8009a/%.cyclo: ../Components/otm8009a/%.c Components/otm8009a/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -DARM_MATH_CM4 -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/Users/alvaromontesano/Documents/STM32CubeIDE/P2_SEMP/STM32F411E-Discovery" -I"/Users/alvaromontesano/Documents/STM32CubeIDE/P2_SEMP/Components" -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Components-2f-otm8009a

clean-Components-2f-otm8009a:
	-$(RM) ./Components/otm8009a/otm8009a.cyclo ./Components/otm8009a/otm8009a.d ./Components/otm8009a/otm8009a.o ./Components/otm8009a/otm8009a.su

.PHONY: clean-Components-2f-otm8009a

