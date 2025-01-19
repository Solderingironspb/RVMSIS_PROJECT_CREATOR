################################################################################
# MRS Version: 1.9.2
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USB_DEVICE/Src/hw_config.c \
../USB_DEVICE/Src/usb_core.c \
../USB_DEVICE/Src/usb_desc.c \
../USB_DEVICE/Src/usb_endp.c \
../USB_DEVICE/Src/usb_init.c \
../USB_DEVICE/Src/usb_int.c \
../USB_DEVICE/Src/usb_istr.c \
../USB_DEVICE/Src/usb_mem.c \
../USB_DEVICE/Src/usb_prop.c \
../USB_DEVICE/Src/usb_pwr.c \
../USB_DEVICE/Src/usb_regs.c \
../USB_DEVICE/Src/usb_sil.c \
../USB_DEVICE/Src/usbd_compatibility_hid.c 

OBJS += \
./USB_DEVICE/Src/hw_config.o \
./USB_DEVICE/Src/usb_core.o \
./USB_DEVICE/Src/usb_desc.o \
./USB_DEVICE/Src/usb_endp.o \
./USB_DEVICE/Src/usb_init.o \
./USB_DEVICE/Src/usb_int.o \
./USB_DEVICE/Src/usb_istr.o \
./USB_DEVICE/Src/usb_mem.o \
./USB_DEVICE/Src/usb_prop.o \
./USB_DEVICE/Src/usb_pwr.o \
./USB_DEVICE/Src/usb_regs.o \
./USB_DEVICE/Src/usb_sil.o \
./USB_DEVICE/Src/usbd_compatibility_hid.o 

C_DEPS += \
./USB_DEVICE/Src/hw_config.d \
./USB_DEVICE/Src/usb_core.d \
./USB_DEVICE/Src/usb_desc.d \
./USB_DEVICE/Src/usb_endp.d \
./USB_DEVICE/Src/usb_init.d \
./USB_DEVICE/Src/usb_int.d \
./USB_DEVICE/Src/usb_istr.d \
./USB_DEVICE/Src/usb_mem.d \
./USB_DEVICE/Src/usb_prop.d \
./USB_DEVICE/Src/usb_pwr.d \
./USB_DEVICE/Src/usb_regs.d \
./USB_DEVICE/Src/usb_sil.d \
./USB_DEVICE/Src/usbd_compatibility_hid.d 


# Each subdirectory must supply rules for building sources it contributes
USB_DEVICE/Src/%.o: ../USB_DEVICE/Src/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g3 -DDEBUG=2 -I../Core/Inc -I../USB_DEVICE/Inc -I../Drivers/inc -std=c11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

