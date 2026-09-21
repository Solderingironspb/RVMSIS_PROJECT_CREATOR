#   Makefile for CH32V203C8T6 (HID_Gamepad_example)
#   Автор: Волков Олег
#   Дата создания скрипта: 12.11.2025
#   GitHub: https://github.com/Solderingironspb
#   Группа Вконтакте: https://vk.com/solderingiron.stm32
#   YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
#   RuTube: https://rutube.ru/channel/36234184/
#   Яндекс Дзен: https://dzen.ru/id/622208eed2eb4c6d0cd16749

#Выбор компиляции под определенную систему. 
#OS_IS_LINUX = 1 - Это Linux
#OS_IS_LINUX = 0 - Это Windows
OS_IS_LINUX = 0

#Имя папки пользователя(для Linux)
USER_FOLDER_NAME = Solderingiron

#######################################################################################################################################
# Системные include-директории
#######################################################################################################################################

#Чтоб clangd видел системные include-директории корректно, а не выдумывал их сам, пропишем пути вручную:
ifeq ($(OS_IS_LINUX), 1)
# Настройки для Linux
# Путь до компилятора:
COMPILER_PATH = /home/$(USER_FOLDER_NAME)/dev_tools/CH32_tools/RISC-V_Embedded_GCC12/riscv-wch-elf
# Системные заголовочне файлы компилятора
ISYSTEM_INCLUDE_PATH_1 = /home/$(USER_FOLDER_NAME)/dev_tools/CH32_tools/RISC-V_Embedded_GCC12/riscv-wch-elf/include/
# Внутренние заголовочные файлы GCC, специфичные для компилятора заголовки
ISYSTEM_INCLUDE_PATH_2 = /home/$(USER_FOLcDER_NAME)/dev_tools/CH32_tools/RISC-V_Embedded_GCC12/riscv-wch-elf/include/sys/
else

# Настройки для Windows
# Путь до компилятора:
COMPILER_PATH = C:/dev_tools/CH32_tools/RISC-V_Embedded_GCC12/riscv-none-elf
# Системные заголовочне файлы компилятора
ISYSTEM_INCLUDE_PATH_1 = C:/dev_tools/CH32_tools/RISC-V_Embedded_GCC12/riscv-none-elf/include
# Внутренние заголовочные файлы GCC, специфичные для компилятора заголовки
ISYSTEM_INCLUDE_PATH_2 = C:/dev_tools/CH32_tools/RISC-V_Embedded_GCC12/riscv-none-elf/include/sys
endif

#######################################################################################################################################
# Пути для проекта
#######################################################################################################################################

# GNU Arm Embedded Toolchain:
ifeq ($(OS_IS_LINUX), 1)
# Настройки для Linux
GNU_TOOLCHAIN = /home/$(USER_FOLDER_NAME)/dev_tools/CH32_tools/RISC-V_Embedded_GCC12/bin
GNU_TOOLCHAIN_GCC_PATH = $(GNU_TOOLCHAIN)/riscv-wch-elf-gcc
GNU_TOOLCHAIN_GDB_PATH = $(GNU_TOOLCHAIN)/riscv-wch-elf-gdb
GNU_TOOLCHAIN_SIZE_PATH = $(GNU_TOOLCHAIN)/riscv-wch-elf-size
else
# Настройки для Windows
GNU_TOOLCHAIN = C:/dev_tools/CH32_tools/RISC-V_Embedded_GCC12/bin
GNU_TOOLCHAIN_GCC_PATH = $(GNU_TOOLCHAIN)/riscv-none-elf-gcc.exe
GNU_TOOLCHAIN_GDB_PATH = $(GNU_TOOLCHAIN)/riscv-none-elf-gdb.exe
GNU_TOOLCHAIN_SIZE_PATH = $(GNU_TOOLCHAIN)/riscv-none-elf-size.exe
endif

# OpenOCD:
ifeq ($(OS_IS_LINUX), 1)
# Настройки для Linux
OPEN_OCD_PATH = /home/$(USER_FOLDER_NAME)/dev_tools/CH32_tools/OpenOCD/OpenOCD/
OPEN_OCD_BIN_PATH = $(OPEN_OCD_PATH)/bin/openocd
else
# Настройки для Windows
OPEN_OCD_PATH = C:/dev_tools/CH32_tools/OpenOCD
OPEN_OCD_BIN_PATH = $(OPEN_OCD_PATH)/bin/openocd.exe
endif


# OpenOCD, кофигурационные файлы под WCH-Link и микроконтроллер (здесь все в одном файле):
ifeq ($(OS_IS_LINUX), 1)
# Настройки для Linux
OPEN_OCD_INTERFACE_PATH = $(OPEN_OCD_PATH)/bin/wch-riscv.cfg
OPEN_OCD_TARGET_PATH = 
else
# Настройки для Windows
OPEN_OCD_INTERFACE_PATH = $(OPEN_OCD_PATH)/bin/wch-riscv.cfg
OPEN_OCD_TARGET_PATH = 
endif


# SVD файл для описания периферии микроконтроллера
ifeq ($(OS_IS_LINUX), 1)
# Настройки для Linux
SVD_FILE_PATH =/home/$(USER_FOLDER_NAME)/dev_tools/CH32_tools/WCH_svd/RISC-V/CH32V203/NoneOS/CH32V203xx.svd
else
# Настройки для Windows
SVD_FILE_PATH = C:/dev_tools/CH32_tools/WCH_svd/RISC-V/CH32V203/NoneOS/CH32V203xx.svd
endif

#######################################################################################################################################
# Название проекта
#######################################################################################################################################
TARGET = Clean_project

#######################################################################################################################################
# Build path
#######################################################################################################################################
BUILD_DIR = Debug

#######################################################################################################################################
# Source location (Вводим через пробел, либо с новой строки, но с знаком '\' в конце)
#######################################################################################################################################
# GNU C 
C_SOURCES = \
Core/Src/ch32v203xx_it.c \
Core/Src/ch32v20x_RVMSIS.c \
Core/Src/main.c \
Core/Src/syscalls.c \
Drivers/src/ch32v20x_adc.c \
Drivers/src/ch32v20x_bkp.c \
Drivers/src/ch32v20x_can.c \
Drivers/src/ch32v20x_crc.c \
Drivers/src/ch32v20x_dbgmcu.c \
Drivers/src/ch32v20x_dma.c \
Drivers/src/ch32v20x_exti.c \
Drivers/src/ch32v20x_flash.c \
Drivers/src/ch32v20x_gpio.c \
Drivers/src/ch32v20x_i2c.c \
Drivers/src/ch32v20x_iwdg.c \
Drivers/src/ch32v20x_misc.c \
Drivers/src/ch32v20x_opa.c \
Drivers/src/ch32v20x_pwr.c \
Drivers/src/ch32v20x_rcc.c \
Drivers/src/ch32v20x_rtc.c \
Drivers/src/ch32v20x_spi.c \
Drivers/src/ch32v20x_tim.c \
Drivers/src/ch32v20x_usart.c \
Drivers/src/ch32v20x_wwdg.c \
Drivers/src/core_riscv.c \
USB_DEVICE/Src/hw_config.c \
USB_DEVICE/Src/usb_core.c \
USB_DEVICE/Src/usb_desc.c \
USB_DEVICE/Src/usb_endp.c \
USB_DEVICE/Src/usb_init.c \
USB_DEVICE/Src/usb_int.c \
USB_DEVICE/Src/usb_istr.c \
USB_DEVICE/Src/usb_mem.c \
USB_DEVICE/Src/usb_prop.c \
USB_DEVICE/Src/usb_pwr.c \
USB_DEVICE/Src/usb_regs.c \
USB_DEVICE/Src/usb_sil.c \
USB_DEVICE/Src/usbd_compatibility_hid.c

# Assembly (Обратите внимание, почему-то STM32 выпускает ASM файлы, 
# то с расширением *.S, то c расширением *.s). Это важно. 
# Для унификации переименуйте в *.s (с маленькой буквой)
ASM_SOURCES = \
Core/Startup/startup_ch32v20x_D6.s

#######################################################################################################################################
# Include location (Вводим через пробел, либо с новой строки, но с знаком '\' в конце)
# Пример: C_INCLUDES = Core/Inc Drivers/CMSIS 
#######################################################################################################################################
# C includes
C_INCLUDES = \
Drivers/inc/ \
Core/Inc/ \
USB_DEVICE/Inc/

# Assembly includes
AS_INCLUDES = 

#######################################################################################################################################
# MCU_Settings
#######################################################################################################################################

CPU = -march=rv32imacxw
INTEGER_ABI = -mabi=ilp32
FPU =
FLOAT_ABI = 
SMALL_DATA_LIMIT = -msmall-data-limit=8
SMALL_PROLOGUE_EPILOGUE = -msave-restore
MCU = $(CPU) $(INTEGER_ABI) $(FPU) $(FLOAT_ABI) $(SMALL_DATA_LIMIT) $(SMALL_PROLOGUE_EPILOGUE) 

#######################################################################################################################################
# MCU_GCC_Compiler (CFLAGS) (ASFLAGS)
#######################################################################################################################################

# Defines (Записываются через пробел. Пример: C_DEFS =  DEBUG STM32F103xB)
# GNU C
C_DEFS =  
# Assembly
AS_DEFS = 

# Language standard
LANG_STD = -std=gnu11

# Optimization
#(None "-O0", Optimize for Debug "-Og", Optimize "-O1", Optimize more "-O2",
# Optimize most "-O3", Optimize for size "-Os", Optimize for speed "-Ofast") 
OPT = -O0 

# Debug level(None " ", Minimal "-g1", Default "-g", Maximum "-g3" )
DEBUG = -g3

# compile gcc flags
ASFLAGS = $(MCU) $(DEBUG) $(OPT) $(addprefix -, $(AS_DEFS)) $(AS_INCLUDES)  -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized 
CFLAGS =  $(MCU) $(DEBUG) $(OPT) $(LANG_STD) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
ASFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"

#######################################################################################################################################
# Binaries
#######################################################################################################################################

# PREFIX
ifeq ($(OS_IS_LINUX), 1)
# Настройки для Linux
PREFIX = riscv-wch-elf-
else
# Настройки для Windows
PREFIX = riscv-none-elf-
endif

ifdef GNU_TOOLCHAIN
CC = $(GNU_TOOLCHAIN)/$(PREFIX)gcc
AS = $(GNU_TOOLCHAIN)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GNU_TOOLCHAIN)/$(PREFIX)objcopy
SZ = $(GNU_TOOLCHAIN)/$(PREFIX)size --format=berkeley
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################################################################################################################
# LDFLAGS
#######################################################################################################################################
# link script
LDSCRIPT = \
Link.ld

# libraries
LIBS = -Wl,--start-group -lc -lm -Wl,--end-group
LIBDIR = 
LDFLAGS = $(MCU) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -nostartfiles -Xlinker --gc-sections -Wl,-Map=$(BUILD_DIR)/$(TARGET).map --specs=nano.specs --specs=nosys.specs -Wl,--print-memory-usage -lprintf

##########################################################################
# Disassemble the ELF file
##########################################################################

CROSS_CREATE_LISTING = --all-headers --demangle --disassemble -M xw

disassemble: $(BUILD_DIR)/$(TARGET).elf
	$(GNU_TOOLCHAIN)/$(PREFIX)objdump $(CROSS_CREATE_LISTING) $< > $(BUILD_DIR)/$(TARGET).disasm

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin disassemble


#######################################################################################################################################
# build the application (Можно не трогать этот участок)
#######################################################################################################################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		


#######################################################################################################################################
# update json files (Обновить JSON файлы в соответствие с данным Makefile). Работает под PowerShell (Core, 7+)
#######################################################################################################################################

update-json:
	pwsh -executionpolicy bypass -File .vscode/ps_scripts/Makefile_update_json.ps1 "$(TARGET)" "$(BUILD_DIR)" "$(C_INCLUDES)" "$(C_DEFS)" "$(GNU_TOOLCHAIN_GCC_PATH)" "$(CFLAGS)" "$(GNU_TOOLCHAIN_GDB_PATH)" "$(OPEN_OCD_BIN_PATH)" "$(OPEN_OCD_INTERFACE_PATH) $(OPEN_OCD_TARGET_PATH)" "$(SVD_FILE_PATH)" "$(GNU_TOOLCHAIN_SIZE_PATH)"


#######################################################################################################################################
# update .clang file (Обновить .clang файл в соответствие с данным Makefile). Работает под PowerShell (Core, 7+)
#######################################################################################################################################

update-clang:
	pwsh -executionpolicy bypass -File .vscode/ps_scripts/clangd_update.ps1 "$(COMPILER_PATH)" "$(ISYSTEM_INCLUDE_PATH_1)" "$(ISYSTEM_INCLUDE_PATH_2)"

ifeq ($(OS_IS_LINUX), 1)
# Настройки для Linux
ld_analyzer:
	python3 .vscode/py_scripts/ld_analyzer.py $(LDSCRIPT) json > memory_config.json
else
# Настройки для Windows
ld_analyzer:
	python .vscode/py_scripts/ld_analyzer.py $(LDSCRIPT) json > memory_config.json
endif


#######################################################################################################################################
# clean up (Очистить проект)
#######################################################################################################################################

CLANGD_CACHE = .cache # Кэш clangd

clean:
	rm -rf $(BUILD_DIR) $(CLANGD_CACHE) compile_commands.json memory_config.json



#######################################################################################################################################
# OpenOCD (Записать прошивку на мк, очистить память мк, продолжить работу мк после debug)
#######################################################################################################################################

flash: all
	$(OPEN_OCD_BIN_PATH) -f $(OPEN_OCD_INTERFACE_PATH) -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

erase: 
	$(OPEN_OCD_BIN_PATH) -f $(OPEN_OCD_INTERFACE_PATH) -c "init; reset halt; flash erase_sector 0 0 last; reset; exit"

resume:
	$(OPEN_OCD_BIN_PATH) -f $(OPEN_OCD_INTERFACE_PATH) -c "init; reset halt; resume; exit"


#######################################################################################################################################
# dependencies
#######################################################################################################################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***



