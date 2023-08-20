################################################################################
# MRS Version: {"version":"1.8.5","date":"2023/05/22"}
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ch32v203xx_it.c \
../Core/Src/ch32v20x_RVMSIS.c \
../Core/Src/main.c \
../Core/Src/syscalls.c 

OBJS += \
./Core/Src/ch32v203xx_it.o \
./Core/Src/ch32v20x_RVMSIS.o \
./Core/Src/main.o \
./Core/Src/syscalls.o 

C_DEPS += \
./Core/Src/ch32v203xx_it.d \
./Core/Src/ch32v20x_RVMSIS.d \
./Core/Src/main.d \
./Core/Src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g3 -I../Core/Inc -I../Drivers/inc -std=c11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

