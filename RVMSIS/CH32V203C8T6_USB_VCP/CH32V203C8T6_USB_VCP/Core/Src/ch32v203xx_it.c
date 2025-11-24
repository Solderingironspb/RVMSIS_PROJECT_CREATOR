/*
 * ch32v203xx_it.c
 *
 *  Created on: Aug 17, 2023
 *      Author: Oleg Volkov
 */


#include "ch32v203xx_it.h"

extern volatile uint32_t SysTimer_ms; //Переменная, аналогичная HAL_GetTick()
extern volatile uint32_t Delay_counter_ms; //Счетчик для функции Delay_ms
extern volatile uint32_t Timeout_counter_ms; //Переменная для таймаута функций

/**
 ******************************************************************************
 *  @breif Прерывание по флагу CNTIF
 ******************************************************************************
 */

void SysTick_Handler(void) {
    SysTick->SR &= ~(1 << 0);
    SysTimer_ms++;

    if (Delay_counter_ms) {
        Delay_counter_ms--;
    }
    if (Timeout_counter_ms) {
        Timeout_counter_ms--;
    }
}


