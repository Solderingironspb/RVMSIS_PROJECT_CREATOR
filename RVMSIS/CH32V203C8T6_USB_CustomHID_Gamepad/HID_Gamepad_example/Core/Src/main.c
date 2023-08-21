/*
 * Пример Custom HID (Gamepad) на CH32V203C8T6
 * В файле usb_desc.h можно найти основные определения:
 * - USBD_VID - VID устройства
 * - USBD_PID - PID устройства
 * - USBD_LANGID_STRING - языковой идентификатор
 * - USBD_SIZE_REPORT_DESC - размер репорт десктриптора (данный дестриптор описывает геймпад)
 * - CUSTOM_HID_BINTERVAL - интервал, с которым геймпад будет опрашиваться (в мс)
 * В файле usb_desc.c можно найти основные настройки устройства:
 * - const uint8_t USBD_DeviceDescriptor[USBD_SIZE_DEVICE_DESC] - USB Device Descriptors
 * - const uint8_t USBD_ConfigDescriptor[USBD_SIZE_CONFIG_DESC] - USB Configration Descriptors
 * - const uint8_t USBD_HidRepDesc[USBD_SIZE_REPORT_DESC] - HID Report Descriptor(он описывает наш геймпад: сколько кнопок, осей, и т.д.)
 * - uint8_t StringVendor[64] - Кастомный iManufacturer
 * - uint8_t StringProduct[64] - Кастомный iProduct
 * - const uint8_t USBD_StringSerial[USBD_SIZE_STRING_SERIAL] - Серийный номер устройства
 *
 */
#include "main.h"
#include "stdio.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "usb_prop.h"
#include "usbd_compatibility_hid.h"

extern uint8_t USBD_ENDPx_DataUp(uint8_t endp, uint8_t *pbuf, uint16_t len);

/*Структура нашего геймпада. В точности повторяет HID Report Descriptor*/
typedef struct
    __attribute__((packed)) {
        int8_t x;
        int8_t y;
        int8_t z;
        int8_t rx;
        int8_t ry;
        int8_t rz;
        uint8_t hat_switch;
        uint16_t buttons;
    } USB_Custom_HID_Gamepad;

    USB_Custom_HID_Gamepad Gamepad_data;

    int main(void) {
        RVMSIS_Debug_init(); //Настройка дебага
        RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты
        RVMSIS_SysTick_Timer_init(); //Настройка системного таймера
#ifdef DEBUG_USE
        RVMSIS_USART3_Init();
#endif
        RVMSIS_GPIO_init(GPIOC, 13, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_50_MHZ);
        GPIOC->BSHR = GPIO_BSHR_BS13; //Т.к. диод на отладочной плате имеет инвертированное включение, чтоб его выключить - нужно отправить SET.

        Set_USBConfig();
        USB_Init();
        USB_Interrupts_Config();

        while(1) {
            if( bDeviceState == CONFIGURED ) {
                GPIOC->BSHR = GPIO_BSHR_BR13; //Если диод загорится - устройство USB сконфигурировалось
                Gamepad_data.x++;
                Gamepad_data.y++;
                Gamepad_data.z++;
                Gamepad_data.rx++;
                Gamepad_data.ry++;
                Gamepad_data.rz++;
                Gamepad_data.hat_switch++;
                Gamepad_data.buttons++;
                if (Gamepad_data.buttons > 0x7FF) {
                    Gamepad_data.buttons = 0;
                }
                USBD_ENDPx_DataUp( ENDP2, (uint8_t*)&Gamepad_data, sizeof (Gamepad_data)); //Отправим в буфер значения кнопок геймпада.
            }
        }
    }
