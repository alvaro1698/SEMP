################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/antSisTask.c \
../Core/Src/freertos.c \
../Core/Src/fsm.c \
../Core/Src/ledSisTask.c \
../Core/Src/lsm303_mag.c \
../Core/Src/main.c \
../Core/Src/movTask.c \
../Core/Src/muesAceTask.c \
../Core/Src/muesMagTask.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/antSisTask.o \
./Core/Src/freertos.o \
./Core/Src/fsm.o \
./Core/Src/ledSisTask.o \
./Core/Src/lsm303_mag.o \
./Core/Src/main.o \
./Core/Src/movTask.o \
./Core/Src/muesAceTask.o \
./Core/Src/muesMagTask.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/antSisTask.d \
./Core/Src/freertos.d \
./Core/Src/fsm.d \
./Core/Src/ledSisTask.d \
./Core/Src/lsm303_mag.d \
./Core/Src/main.d \
./Core/Src/movTask.d \
./Core/Src/muesAceTask.d \
./Core/Src/muesMagTask.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -DARM_MATH_CM4=1 -c -I../Core/Inc -I../Middlewares/ST/ARM/DSP/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/antSisTask.cyclo ./Core/Src/antSisTask.d ./Core/Src/antSisTask.o ./Core/Src/antSisTask.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/fsm.cyclo ./Core/Src/fsm.d ./Core/Src/fsm.o ./Core/Src/fsm.su ./Core/Src/ledSisTask.cyclo ./Core/Src/ledSisTask.d ./Core/Src/ledSisTask.o ./Core/Src/ledSisTask.su ./Core/Src/lsm303_mag.cyclo ./Core/Src/lsm303_mag.d ./Core/Src/lsm303_mag.o ./Core/Src/lsm303_mag.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/movTask.cyclo ./Core/Src/movTask.d ./Core/Src/movTask.o ./Core/Src/movTask.su ./Core/Src/muesAceTask.cyclo ./Core/Src/muesAceTask.d ./Core/Src/muesAceTask.o ./Core/Src/muesAceTask.su ./Core/Src/muesMagTask.cyclo ./Core/Src/muesMagTask.d ./Core/Src/muesMagTask.o ./Core/Src/muesMagTask.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

