/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_endp.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/08/08
 * Description        : Endpoint routines
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "usb_prop.h"

uint8_t USBD_Endp3_Busy;
uint16_t USB_Rx_Cnt = 0;

/*********************************************************************
 * @fn      EP2_IN_Callback
 *
 * @brief  Endpoint 1 IN.
 *
 * @return  none
 */
void EP1_IN_Callback(void) {

}

/*********************************************************************
 * @fn      EP2_OUT_Callback
 *
 * @brief  Endpoint 2 OUT.
 *
 * @return  none
 */

uint8_t USB_Buffer[64] = { 0, };
uint32_t USB_Buffer_len;
void EP2_OUT_Callback(void) {

    USB_Buffer_len = GetEPRxCount( EP2_OUT & 0x7F);
    if (USB_Buffer_len < 64) {
        /*Cкопируем приходящие данные в USB_Buffer. USB_Buffer_len - размер приходящих данных*/
        PMAToUserBufferCopy(USB_Buffer, GetEPRxAddr( EP2_OUT & 0x7F), USB_Buffer_len);
        SetEPRxValid( ENDP2);
        /*Отправим обратно то, то получим*/
        USBD_ENDPx_DataUp( ENDP3, USB_Buffer, USB_Buffer_len);
    } else {
        PMAToUserBufferCopy(USB_Buffer, GetEPRxAddr( EP2_OUT & 0x7F), 64);
        SetEPRxValid( ENDP2);
    }

    SetEPRxValid( ENDP2);

}
/*********************************************************************
 * @fn      EP3_IN_Callback
 *
 * @brief  Endpoint 3 IN.
 *
 * @return  none
 */
void EP3_IN_Callback(void) {
    USBD_Endp3_Busy = 0;
    //Uart.USB_Up_IngFlag = 0x00;
}

/*********************************************************************
 * @fn      USBD_ENDPx_DataUp
 *
 * @brief  USBD ENDPx DataUp Function
 * 
 * @param   endp - endpoint num.
 *          *pbuf - A pointer points to data.
 *          len - data length to transmit.
 * 
 * @return  data up status.
 */
uint8_t USBD_ENDPx_DataUp(uint8_t endp, uint8_t *pbuf, uint16_t len) {
    if (endp == ENDP3) {
        if (USBD_Endp3_Busy) {
            return USB_ERROR;
        }
        USB_SIL_Write( EP3_IN, pbuf, len);
        USBD_Endp3_Busy = 1;
        SetEPTxStatus( ENDP3, EP_TX_VALID);
    } else {
        return USB_ERROR;
    }
    return USB_SUCCESS;
}
