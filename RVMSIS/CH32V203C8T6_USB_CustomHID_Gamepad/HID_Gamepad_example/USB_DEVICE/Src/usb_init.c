/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_init.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/08/08
 * Description        : Initialization routines & global variables
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "usb_lib.h"
#include "usb_desc.h"


extern uint8_t StringVendor[64];
extern uint8_t StringProduct[64];
extern uint8_t USBD_StringVendor [130];
extern uint8_t USBD_StringProduct[130];
extern ONE_DESCRIPTOR String_Descriptor[4];

uint8_t EPindex;
DEVICE_INFO *pInformation;
DEVICE_PROP *pProperty;
uint16_t SaveState;
uint16_t wInterrupt_Mask;
DEVICE_INFO Device_Info;
USER_STANDARD_REQUESTS *pUser_Standard_Requests;

/*******************************************************************************
 * @fn        USB_Init
 *
 * @brief     USB system initialization
 *
 * @return    None.
 *
 */
void USB_Init(void) {
    String_Descriptor[1].Descriptor_Size = USB_String_data_init(StringVendor, USBD_StringVendor, USB_STRING_DESCRIPTOR_TYPE);
    String_Descriptor[2].Descriptor_Size = USB_String_data_init(StringProduct, USBD_StringProduct, USB_STRING_DESCRIPTOR_TYPE);

    pInformation = &Device_Info;
    pInformation->ControlState = 2;
    pProperty = &Device_Property;
    pUser_Standard_Requests = &User_Standard_Requests;
    pProperty->Init();
}

