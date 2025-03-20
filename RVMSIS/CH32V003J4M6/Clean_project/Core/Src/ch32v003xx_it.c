/*
 * ch32v003_it.c
 *
 *  Created on: May 16, 2024
 *      Author: Solderingiron
 */

#include "ch32v003xx_it.h"
#include "main.h"

/**
 ***************************************************************************************
 *  @breif Настройка Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */
extern volatile uint32_t SysTimer_ms;         // Переменная, аналогичная HAL_GetTick()
extern volatile uint32_t Delay_counter_ms;    // Счетчик для функции Delay_ms
extern volatile uint32_t Timeout_counter_ms;  // Переменная для таймаута функций

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
