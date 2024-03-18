################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include STM32F411E-Discovery/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Components/wm8994/subdir.mk
-include Components/ts3510/subdir.mk
-include Components/stmpe811/subdir.mk
-include Components/stmpe1600/subdir.mk
-include Components/st7789h2/subdir.mk
-include Components/st7735/subdir.mk
-include Components/s5k5cag/subdir.mk
-include Components/ov5640/subdir.mk
-include Components/ov2640/subdir.mk
-include Components/otm8009a/subdir.mk
-include Components/nt35510/subdir.mk
-include Components/mfxstm32l152/subdir.mk
-include Components/lsm303dlhc/subdir.mk
-include Components/lsm303agr/subdir.mk
-include Components/ls016b8uy/subdir.mk
-include Components/lis3dsh/subdir.mk
-include Components/lis302dl/subdir.mk
-include Components/lan8742/subdir.mk
-include Components/l3gd20/subdir.mk
-include Components/ili9341/subdir.mk
-include Components/ili9325/subdir.mk
-include Components/i3g4250d/subdir.mk
-include Components/ft6x06/subdir.mk
-include Components/ft3x67/subdir.mk
-include Components/exc7200/subdir.mk
-include Components/dp83848/subdir.mk
-include Components/cs43l22/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := P3__FreeRTos
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
P3__FreeRTos.elf \

MAP_FILES += \
P3__FreeRTos.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
P3__FreeRTos.list \


# All Target
all: main-build

# Main-build Target
main-build: P3__FreeRTos.elf secondary-outputs

# Tool invocations
P3__FreeRTos.elf P3__FreeRTos.map: $(OBJS) $(USER_OBJS) /Users/alvaromontesano/Documents/STM32CubeIDE/P3__FreeRTos/STM32F411VETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "P3__FreeRTos.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/alvaromontesano/Documents/STM32CubeIDE/P3__FreeRTos/STM32F411VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="P3__FreeRTos.map" -Wl,--gc-sections -static -L../Middlewares/ST/ARM/DSP/Lib -L../Middlewares/ST/ARM/DSP/Inc --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

P3__FreeRTos.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "P3__FreeRTos.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) P3__FreeRTos.elf P3__FreeRTos.list P3__FreeRTos.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
