#   Makefile for CH32V003J4M6
#   Автор: Волков Олег
#   Дата создания скрипта: 20.03.2025
#   GitHub: https://github.com/Solderingironspb
#   Группа Вконтакте: https://vk.com/solderingiron.stm32
#   YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
#   RuTube: https://rutube.ru/channel/36234184/
#   Яндекс Дзен: https://dzen.ru/id/622208eed2eb4c6d0cd16749

######################################################################################
# Пути для проекта
######################################################################################
# GNU RISC-V Embedded Toolchain:
GNU_TOOLCHAIN = C:/dev_tools/CH32_tools/RISC-V_Embedded_GCC12/bin
GNU_TOOLCHAIN_GCC_PATH = $(GNU_TOOLCHAIN)/riscv-none-elf-gcc.exe
GNU_TOOLCHAIN_GDB_PATH = $(GNU_TOOLCHAIN)/riscv-none-elf-gdb.exe
GNU_TOOLCHAIN_SIZE_PATH = $(GNU_TOOLCHAIN)/riscv-none-elf-size.exe

# OpenOCD:
OPEN_OCD_PATH = C:/dev_tools/CH32_tools/OpenOCD
OPEN_OCD_BIN_PATH = $(OPEN_OCD_PATH)/bin/openocd.exe

# OpenOCD, кофигурационные файлы под WCH-Link и микроконтроллер (здесь все в одном файле):
OPEN_OCD_INTERFACE_PATH = $(OPEN_OCD_PATH)/bin/wch-riscv.cfg
OPEN_OCD_TARGET_PATH = 

# SVD файл для описания периферии микроконтроллера
SVD_FILE_PATH = C:/dev_tools/CH32_tools/WCH_svd/RISC-V/CH32V003/NoneOS/CH32V003xx.svd

#####################################################################################
# Название проекта
#####################################################################################
TARGET = Clean_project

#####################################################################################
# Build path
#####################################################################################
BUILD_DIR = Debug

#####################################################################################
# Source location (Вводим через пробел, либо с новой строки, но с знаком '\' в конце)
#####################################################################################
# GNU C 
C_SOURCES = \
Core/Src/ch32v003xx_it.c \
Core/Src/ch32v00x_RVMSIS.c \
Core/Src/main.c \
Drivers/Src/ch32v00x_adc.c \
Drivers/Src/ch32v00x_dbgmcu.c \
Drivers/Src/ch32v00x_dma.c \
Drivers/Src/ch32v00x_exti.c \
Drivers/Src/ch32v00x_flash.c \
Drivers/Src/ch32v00x_gpio.c \
Drivers/Src/ch32v00x_i2c.c \
Drivers/Src/ch32v00x_iwdg.c \
Drivers/Src/ch32v00x_misc.c \
Drivers/Src/ch32v00x_opa.c \
Drivers/Src/ch32v00x_pwr.c \
Drivers/Src/ch32v00x_rcc.c \
Drivers/Src/ch32v00x_spi.c \
Drivers/Src/ch32v00x_tim.c \
Drivers/Src/ch32v00x_usart.c \
Drivers/Src/ch32v00x_wwdg.c \
Drivers/Src/core_riscv.c
# Assembly (Обратите внимание, почему-то STM32 и CH32 выпускает ASM файлы, 
# то с расширением *.S, то c расширением *.s). Это важно. 
# Для унификации переименуйте в *.s (с маленькой буквой)
ASM_SOURCES = \
Startup/startup_ch32v00x.s

#####################################################################################
# Include location (Вводим через пробел, либо с новой строки, но с знаком '\' в конце)
# Пример: C_INCLUDES = Core/Inc Drivers/CMSIS 
#####################################################################################
# C includes
C_INCLUDES = \
Core/Inc/ \
Drivers/Inc/
# Assembly includes
AS_INCLUDES = 

#####################################################################################
# MCU_Settings
#####################################################################################

CPU = -march=rv32ecxw
INTEGER_ABI = -mabi=ilp32e
FPU =
FLOAT_ABI = 
SMALL_DATA_LIMIT = -msmall-data-limit=0
SMALL_PROLOGUE_EPILOGUE = -msave-restore
MCU = $(CPU) $(INTEGER_ABI) $(FPU) $(FLOAT_ABI) $(SMALL_DATA_LIMIT) $(SMALL_PROLOGUE_EPILOGUE) 

#####################################################################################
# MCU_GCC_Compiler (CFLAGS) (ASFLAGS)
#####################################################################################

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

#####################################################################################
# Binaries
#####################################################################################
PREFIX = riscv-none-elf-

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

#####################################################################################
# LDFLAGS
#####################################################################################
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


#####################################################################################
# build the application (Можно не трогать этот участок)
#####################################################################################
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




##########################################################################
# update json files (Обновить JSON файлы в соответствие с данным Makefile)
##########################################################################

update-json:
	powershell.exe -executionpolicy bypass -File .vscode/ps_scripts/Makefile_update_json.ps1 "$(TARGET)" "$(BUILD_DIR)" "$(C_INCLUDES)" "$(C_DEFS)" "$(GNU_TOOLCHAIN_GCC_PATH)" "$(CFLAGS)" "$(GNU_TOOLCHAIN_GDB_PATH)" "$(OPEN_OCD_BIN_PATH)" "$(OPEN_OCD_INTERFACE_PATH) $(OPEN_OCD_TARGET_PATH)" "$(SVD_FILE_PATH)" "$(GNU_TOOLCHAIN_SIZE_PATH)"

##########################################################################
# clean up (for Windows)
##########################################################################
clean:
	rmdir /S /Q $(BUILD_DIR)

#########################################################################
# OpenOCD (Записать прошивку на мк и очистить память мк)
##########################################################################
flash: all
	$(OPEN_OCD_BIN_PATH) -f $(OPEN_OCD_INTERFACE_PATH) -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

erase:
	$(OPEN_OCD_BIN_PATH) -f $(OPEN_OCD_INTERFACE_PATH) -c "init; reset halt; flash erase_sector 0 0 last; reset; exit"

resume:
	$(OPEN_OCD_BIN_PATH) -f $(OPEN_OCD_INTERFACE_PATH) -c "init; reset halt; resume; exit"
	
##########################################################################
# dependencies
##########################################################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***