/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_desc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/08/08
 * Description        : This file contains all the functions prototypes for the  
 *                      USB description firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/ 
#ifndef __USB_DESC_H
#define __USB_DESC_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "ch32v20x.h"
	 
	 
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define DEF_USBD_UEP0_SIZE          64           
#define DEF_USBD_MAX_PACK_SIZE      64
       
#define USBD_SIZE_DEVICE_DESC        18
#define USBD_SIZE_CONFIG_DESC        67
#define USBD_SIZE_STRING_LANGID      4
#define USBD_SIZE_STRING_SERIAL      22
#define CUSTOM_HID_BINTERVAL         0x1

#define USBD_VID                     0x1A86
#define USBD_PID                     0xFE0C
#define USBD_LANGID_STRING           1033

extern const uint8_t USBD_DeviceDescriptor[USBD_SIZE_DEVICE_DESC];
extern const uint8_t USBD_ConfigDescriptor[USBD_SIZE_CONFIG_DESC];

extern const uint8_t USBD_StringLangID [USBD_SIZE_STRING_LANGID];
extern uint8_t USBD_StringVendor [130];
extern uint8_t USBD_StringProduct[130];
extern uint8_t USBD_StringSerial [USBD_SIZE_STRING_SERIAL];
uint8_t USB_String_data_init(uint8_t *String_data, uint8_t *Array, uint8_t Descriptor_type);

#ifdef __cplusplus
}
#endif

#endif /* __USB_DESC_H */
