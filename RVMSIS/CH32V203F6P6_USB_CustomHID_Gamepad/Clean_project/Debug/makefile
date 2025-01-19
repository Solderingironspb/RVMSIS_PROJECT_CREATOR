################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Src/subdir.mk
-include Startup/subdir.mk
-include Drivers/src/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
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

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
Blink.hex \

SECONDARY_LIST += \
Blink.lst \

SECONDARY_SIZE += \
Blink.siz \


# All Target
all: Blink.elf secondary-outputs

# Tool invocations
Blink.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -Wl,--print-memory-usage -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g3 -T "C:\Users\Solderingiron\Desktop\Blink\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"Blink.map" --specs=nano.specs --specs=nosys.specs -o "Blink.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@	@
Blink.hex: Blink.elf
	@	riscv-none-embed-objcopy -O ihex "Blink.elf"  "Blink.hex"
	@	@
Blink.lst: Blink.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble -M xw "Blink.elf" > "Blink.lst"
	@	@
Blink.siz: Blink.elf
	@	riscv-none-embed-size --format=berkeley "Blink.elf"
	@	@
# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) Blink.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
