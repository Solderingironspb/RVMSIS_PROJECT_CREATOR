/*
 * Пример Virtual Vom Port на CH32V203C8T6
 * В файле usb_desc.h можно найти основные определения:
 * - USBD_VID - VID устройства
 * - USBD_PID - PID устройства
 * - USBD_LANGID_STRING - языковой идентификатор
 * - CUSTOM_HID_BINTERVAL - интервал, с которым устройство будет опрашиваться (в мс)
 * В файле usb_desc.c можно найти основные настройки устройства:
 * - const uint8_t USBD_DeviceDescriptor[USBD_SIZE_DEVICE_DESC] - USB Device Descriptors
 * - const uint8_t USBD_ConfigDescriptor[USBD_SIZE_CONFIG_DESC] - USB Configration Descriptors
 * - uint8_t StringVendor[64] - Кастомный iManufacturer
 * - uint8_t StringProduct[64] - Кастомный iProduct
 * - const uint8_t USBD_StringSerial[USBD_SIZE_STRING_SERIAL] - Серийный номер устройства
 * В файле usb_endp.c
 * - uint8_t USB_Buffer[64] = { 0, }; - Буфер под входящие данные
 * - uint32_t USB_Buffer_len; - Длина входящих данных
 * - void EP2_OUT_Callback(void) - здесь мы принимаем данные
 * -  USBD_ENDPx_DataUp( ENDP3, USB_Buffer, USB_Buffer_len); - отправка данных
 * В примере настроено все так: открываем терминал. Что туда напишем - то нам МК и вернет. Буфер 64 байта.
 */

#include "main.h"
#include "stdio.h"
#include "usb_lib.h"
#include "usb_pwr.h"

#define DEBUG_USE   //Использовать DEBUG по USART

int main(void) {
    RVMSIS_Debug_init(); //Настройка дебага
    RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты
    RVMSIS_SysTick_Timer_init(); //Настройка системного таймера
    RVMSIS_USART3_Init(); //См. файл syscalls.c USART3 115200 8N1 выбран для отладки через printf. Ножка PB10
    RVMSIS_GPIO_init(GPIOC, 13, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_50_MHZ);
    GPIOC->BSHR = GPIO_BSHR_BS13;//Т.к. диод на отладочной плате имеет инвертированное включение, чтоб его выключить - нужно отправить SET.

    Set_USBConfig();
    USB_Init();
    USB_Interrupts_Config();

#ifdef DEBUG_USE
    printf("Hello world!\r\n");
#endif

    while(1) {
        if( bDeviceState == CONFIGURED ) {
            GPIOC->BSHR = GPIO_BSHR_BR13;//Если диод загорится - устройство USB сконфигурировалось
        }
    }

}

