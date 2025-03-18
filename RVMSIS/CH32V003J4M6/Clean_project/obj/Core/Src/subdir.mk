################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ch32v003xx_it.c \
../Core/Src/ch32v00x_RVMSIS.c \
../Core/Src/main.c 

OBJS += \
./Core/Src/ch32v003xx_it.o \
./Core/Src/ch32v00x_RVMSIS.o \
./Core/Src/main.o 

C_DEPS += \
./Core/Src/ch32v003xx_it.d \
./Core/Src/ch32v00x_RVMSIS.d \
./Core/Src/main.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g3 -I"C:\Users\Solderingiron\Documents\GitHub\RVMSIS_PROJECT_CREATOR\RVMSIS\CH32V003J4M6\Clean_project\Core\Inc" -I"C:\Users\Solderingiron\Documents\GitHub\RVMSIS_PROJECT_CREATOR\RVMSIS\CH32V003J4M6\Clean_project\Drivers\inc" -std=c11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

