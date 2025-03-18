################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../Core/Startup/startup_ch32v00x.S 

OBJS += \
./Core/Startup/startup_ch32v00x.o 

S_UPPER_DEPS += \
./Core/Startup/startup_ch32v00x.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.S
	@	@	riscv-none-embed-gcc -march=rv32ecxw -mabi=ilp32e -msmall-data-limit=0 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g3 -x assembler-with-cpp -I"C:\Users\Solderingiron\Documents\GitHub\RVMSIS_PROJECT_CREATOR\RVMSIS\CH32V003J4M6\Clean_project\Core\Startup" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

