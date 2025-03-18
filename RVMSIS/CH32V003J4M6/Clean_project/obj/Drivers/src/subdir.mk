################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/src/ch32v00x_adc.c \
../Drivers/src/ch32v00x_dbgmcu.c \
../Drivers/src/ch32v00x_dma.c \
../Drivers/src/ch32v00x_exti.c \
../Drivers/src/ch32v00x_flash.c \
../Drivers/src/ch32v00x_gpio.c \
../Drivers/src/ch32v00x_i2c.c \
../Drivers/src/ch32v00x_iwdg.c \
../Drivers/src/ch32v00x_misc.c \
../Drivers/src/ch32v00x_opa.c \
../Drivers/src/ch32v00x_pwr.c \
../Drivers/src/ch32v00x_rcc.c \
../Drivers/src/ch32v00x_spi.c \
../Drivers/src/ch32v00x_tim.c \
../Drivers/src/ch32v00x_usart.c \
../Drivers/src/ch32v00x_wwdg.c \
../Drivers/src/core_riscv.c 

OBJS += \
./Drivers/src/ch32v00x_adc.o \
./Drivers/src/ch32v00x_dbgmcu.o \
./Drivers/src/ch32v00x_dma.o \
./Drivers/src/ch32v00x_exti.o \
./Drivers/src/ch32v00x_flash.o \
./Drivers/src/ch32v00x_gpio.o \
./Drivers/src/ch32v00x_i2c.o \
./Drivers/src/ch32v00x_iwdg.o \
./Drivers/src/ch32v00x_misc.o \
./Drivers/src/ch32v00x_opa.o \
./Drivers/src/ch32v00x_pwr.o \
./Drivers/src/ch32v00x_rcc.o \
./Drivers/src/ch32v00x_spi.o \
./Drivers/src/ch32v00x_tim.o \
./Drivers/src/ch32v00x_usart.o \
./Drivers/src/ch32v00x_wwdg.o \
./Drivers/src/core_riscv.o 

C_DEPS += \
./Drivers/src/ch32v00x_adc.d \
./Drivers/src/ch32v00x_dbgmcu.d \
./Drivers/src/ch32v00x_dma.d \
./Drivers/src/ch32v00x_exti.d \
./Drivers/src/ch32v00x_flash.d \
./Drivers/src/ch32v00x_gpio.d \
./Drivers/src/ch32v00x_i2c.d \
./Drivers/src/ch32v00x_iwdg.d \
./Drivers/src/ch32v00x_misc.d \
./Drivers/src/ch32v00x_opa.d \
./Drivers/src/ch32v00x_pwr.d \
./Drivers/src/ch32v00x_rcc.d \
./Drivers/src/ch32v00x_spi.d \
./Drivers/src/ch32v00x_tim.d \
./Drivers/src/ch32v00x_usart.d \
./Drivers/src/ch32v00x_wwdg.d \
./Drivers/src/core_riscv.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/src/%.o: ../Drivers/src/%.c
	@	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g3 -I"C:\Users\Solderingiron\Documents\GitHub\RVMSIS_PROJECT_CREATOR\RVMSIS\CH32V003J4M6\Clean_project\Core\Inc" -I"C:\Users\Solderingiron\Documents\GitHub\RVMSIS_PROJECT_CREATOR\RVMSIS\CH32V003J4M6\Clean_project\Drivers\inc" -std=c11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

