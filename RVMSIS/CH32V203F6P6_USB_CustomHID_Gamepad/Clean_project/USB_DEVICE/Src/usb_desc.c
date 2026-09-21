/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_desc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/07/15
 * Description        : USB Descriptors.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "usb_lib.h"
#include "usb_desc.h"
#include <string.h>

/*§©§Ñ§á§à§Ý§ß§Ö§ß§Ú§Ö §Þ§Ñ§ã§ã§Ú§Ó§Ñ §ã §Ü§Ñ§ã§ä§à§Þ§ß§í§Þ iManufacturer §Ú iProduct (§ã§Þ. §æ§Ñ§Û§Ý usb_init.c §æ§å§ß§Ü§è§Ú§ñ void USB_Init(void))*/
uint8_t USB_String_data_init(uint8_t *String_data, uint8_t *Array, uint8_t Descriptor_type) {
    uint8_t Size = 0;
    Size = strlen((char*) String_data);
    Array[0] = (Size * 2) + 2;
    Array[1] = Descriptor_type;
    for (uint8_t i = 0; i < Size; i++) {
        Array[2 + i * 2] = String_data[i];
        Array[2 + i * 2 + 1] = 0;
    }
    return Array[0];
}

/* USB Device Descriptors */
const uint8_t USBD_DeviceDescriptor[USBD_SIZE_DEVICE_DESC] = {
USBD_SIZE_DEVICE_DESC,           // bLength
        0x01,                            // bDescriptorType (Device)
        0x10, 0x01,                      // bcdUSB 1.10
        0x00,                            // bDeviceClass (Use class information in the Interface Descriptors)
        0x00,                            // bDeviceSubClass
        0x00,                            // bDeviceProtocol
        DEF_USBD_UEP0_SIZE,              // bMaxPacketSize0 8
        LOBYTE(USBD_VID), /*idVendor*/
        HIBYTE(USBD_VID), /*idVendor*/
        LOBYTE(USBD_PID), /*idProduct*/
        HIBYTE(USBD_PID), /*idProduct*/
        0x00, 0x01,                      // bcdDevice 2.00
        0x01,                            // iManufacturer (String Index)
        0x02,                            // iProduct (String Index)
        0x03,                            // iSerialNumber (String Index)
        0x01                            // bNumConfigurations 1
        };

/* USB Configration Descriptors */
const uint8_t USBD_ConfigDescriptor[USBD_SIZE_CONFIG_DESC] = {
/* Configuration Descriptor */
0x09,                           // bLength
        0x02,                           // bDescriptorType
        USBD_SIZE_CONFIG_DESC & 0xFF, USBD_SIZE_CONFIG_DESC >> 8, // wTotalLength
        0x01,                           // bNumInterfaces
        0x01,                           // bConfigurationValue
        0x03,                           // iConfiguration (String Index)
        0x80,                           // bmAttributes Remote Wakeup
        0x32,                           // bMaxPower 100mA

        /* Interface Descriptor */
        0x09,                           // bLength
        0x04,                           // bDescriptorType (Interface)
        0x00,                           // bInterfaceNumber 0
        0x00,                           // bAlternateSetting
        0x02,                           // bNumEndpoints 2
        0x03,                           // bInterfaceClass
        0x00,                           // bInterfaceSubClass
        0x00,                           // bInterfaceProtocol
        0x00,                           // iInterface (String Index)

        /* HID Descriptor */
        0x09,                           // bLength
        0x21,                           // bDescriptorType
        0x11, 0x01,                     // bcdHID
        0x00,                           // bCountryCode
        0x01,                           // bNumDescriptors
        0x22,                           // bDescriptorType
        USBD_SIZE_REPORT_DESC & 0xFF, USBD_SIZE_REPORT_DESC >> 8, // wDescriptorLength

        /* Endpoint Descriptor */
        0x07,                           // bLength
        0x05,                           // bDescriptorType
        0x01,                           // bEndpointAddress: OUT Endpoint 1
        0x03,                           // bmAttributes
        0x40, 0x00,                     // wMaxPacketSize
        CUSTOM_HID_BINTERVAL,                           // bInterval: 5mS

        /* Endpoint Descriptor */
        0x07,                           // bLength
        0x05,                           // bDescriptorType
        0x82,                           // bEndpointAddress: IN Endpoint 2
        0x03,                           // bmAttributes
        0x40, 0x00,                     // wMaxPacketSize
        CUSTOM_HID_BINTERVAL                           // bInterval: 5mS
        };

/* USB String Descriptors */
const uint8_t USBD_StringLangID[USBD_SIZE_STRING_LANGID] = {
USBD_SIZE_STRING_LANGID,
USB_STRING_DESCRIPTOR_TYPE, LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING) };

uint8_t StringVendor[64] = "Solderingiron spb 2023";
uint8_t StringProduct[64] = "My Custom HID Gamepad";
uint8_t USBD_Size_String_Vendor;
uint8_t USBD_Size_String_Product;

/* USB Device String Vendor */
uint8_t USBD_StringVendor[130] = {
14,
USB_STRING_DESCRIPTOR_TYPE, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0 }; //§á§à-§å§Þ§à§Ý§é§Ñ§ß§Ú§ð. §£§á§Ú§ê§Ú§ä§Ö §ã§Ó§à§Ö §ß§Ñ§Ù§Ó§Ñ§ß§Ú§Ö §Ó StringVendor[64]

/* USB Device String Product */
uint8_t USBD_StringProduct[130] = {
40,
USB_STRING_DESCRIPTOR_TYPE, 'C', 0, 'H', 0, '3', 0, '2', 0, 'V', 0, '2', 0, '0', 0, 'x', 0, '-', 0, 'C', 0, 'u', 0,'s', 0, 't', 0, 'o', 0, 'm', 0, ' ', 0, 'H', 0, 'I', 0, 'D', 0 }; //§á§à-§å§Þ§à§Ý§é§Ñ§ß§Ú§ð. §£§á§Ú§ê§Ú§ä§Ö §ã§Ó§à§Ö §ß§Ñ§Ù§Ó§Ñ§ß§Ú§Ö §Ó StringProduct[64]

/* USB Device String Serial */
const uint8_t USBD_StringSerial[USBD_SIZE_STRING_SERIAL] = {
USBD_SIZE_STRING_SERIAL,
USB_STRING_DESCRIPTOR_TYPE, '0', 0, '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0, '9', 0 };

/* HID Report Descriptor */
const uint8_t USBD_HidRepDesc[USBD_SIZE_REPORT_DESC] = { 0x05, 0x01, //USAGE_PAGE (Generic Desktop)
        0x09, 0x05, //USAGE (Game Pad)
        0xA1, 0x01, //COLLECTION (Application)
        0x09, 0x30, //USAGE (X)
        0x09, 0x31, //USAGE (Y)
        0x09, 0x32, //USAGE (Z)
        0x09, 0x33, //USAGE (Rx)
        0x09, 0x34, //USAGE (Ry)
        0x09, 0x35, //USAGE (Rz)
        0x15, 0x80, //LOGICAL_MINIMUM (-128)
        0x25, 0x7F, //LOGICAL_MAXIMUM (127)
        0x95, 0x06, //REPORT_COUNT (6)
        0x75, 0x08, //REPORT_SIZE (8)
        0x81, 0x02, //INPUT (Data, Var, Abs)
        0x09, 0x39, //USAGE (Hat swich)
        0x65, 0x14, //UNIT (eng Rot:Angular Pos)
        0x15, 0x00, //LOGICAL_MINIMUM (0)
        0x25, 0x07, //LOGICAL_MAXIMUM (7)
        0x35, 0x00, //PHYSICAL_MINIMUM (00)
        0x46, 0x3B, 0x01, //PHYSICAL_MAXIMUM (315)
        0x95, 0x01, //REPOPRT_COUNT (1)
        0x75, 0x04, //REPORT_SIZE (4)
        0x81, 0x02, //INPUT (Data, Var, Abs)
        0x95, 0x04, //REPORT_COUNT(4)
        0x75, 0x01, //REPORT_SIZE (1)
        0x81, 0x01, //INPUT (Cnst, Ary, Abs)
        0x05, 0x09, //USAGE_PAGE (Button)
        0x19, 0x01, //USAGE_MINIMUM (Button 1)
        0x29, 0x0b, //USAGE_MAXIMUM (Button 11)
        0x15, 0x00, //LOGICAL_MINIMUM (0)
        0x25, 0x01, //LOGICAL_MAXIMUM (1)
        0x75, 0x01, //REPORT_SIZE (1)
        0x95, 0x0B, //REPORT_COUNT (11)
        0x81, 0x02, //INPUT (Data, Var, Abs)
        0x95, 0x05, //REPORT_COUNT (5)
        0x75, 0x01, //REPORT_SIZE (1)
        0x81, 0x01, //INPUT (Cnst, Ary, Abs)
        0xC0 /*     END_COLLECTION              */ //76
        };

