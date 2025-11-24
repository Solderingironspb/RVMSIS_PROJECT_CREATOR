/********************************** (C) COPYRIGHT *******************************
 * File Name  :usbd_custom_hid.c
 * Author     :OWNER
 * Version    : v0.01
 * Date       : 2022��7��8��
 * Description:
*******************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v20x.h"
#include "usbd_compatibility_hid.h"

__attribute__ ((aligned(4))) uint8_t HID_Report_Buffer[DEF_USBD_MAX_PACK_SIZE];   // HID Report Buffer






