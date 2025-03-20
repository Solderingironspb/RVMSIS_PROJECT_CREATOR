/*
 * ch32v00x_RVMSIS.h
 *
 *  Created on: May 16, 2024
 *      Author: Solderingiron
 */

#ifndef INC_CH32V00X_RVMSIS_H_
#define INC_CH32V00X_RVMSIS_H_

#include "ch32v00x.h"
#include "main.h"

#define __WEAK __attribute__((weak))
#define __INTERRUPTF __attribute__((interrupt("WCH-Interrupt-fast")))
#define __INTERRUPTM __attribute__((interrupt("machine")))

// GPIO Configuration mode
enum {
    GPIO_GENERAL_PURPOSE_OUTPUT,
    GPIO_ALTERNATIVE_FUNCTION_OUTPUT,
    GPIO_INPUT
};
// GPIO Input/OUTPUT type
enum {
    GPIO_OUTPUT_PUSH_PULL,
    GPIO_OUTPUT_OPEN_DRAIN,
    GPIO_INPUT_ANALOG,
    GPIO_INPUT_FLOATING,
    GPIO_INPUT_PULL_DOWN,
    GPIO_INPUT_PULL_UP
};

// GPIO Maximum output speed
enum {
    GPIO_SPEED_RESERVED,
    GPIO_SPEED_10_MHZ,
    GPIO_SPEED_2_MHZ,
    GPIO_SPEED_50_MHZ
};

#define USART_MAX_LEN_RX_BUFFER 20

// Структура по USART
struct USART_name {
    uint8_t tx_buffer[20];  // Буфер под выходящие данные
    uint8_t rx_buffer[20];  // Буфер под входящие данные
    uint16_t rx_counter;    // Счетчик приходящих данных типа uint8_t по USART
    uint16_t rx_len;        // Количество принятых байт после сработки флага IDLE
};

void RVMSIS_RCC_SystemClock_48MHz(void);
void RVMSIS_SysTick_Timer_init(void);
void Delay_ms(uint32_t Milliseconds);
void RVMSIS_GPIO_init(GPIO_TypeDef* GPIO, uint8_t GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Speed);
void RVMSIS_TIM1_init(void);
void RVMSIS_TIM1_PWM_CHANNEL1_init(void);
void RVMSIS_TIM1_PWM_CHANNEL2_init(void);
void RVMSIS_TIM2_init(void);
void RVMSIS_USART1_Init(void);
__attribute__((interrupt("machine"))) void USART1_IRQHandler(void);
bool RVMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms);
void RVMSIS_TIM2_PWM_CHANNEL3_init(void);
void RVMSIS_TIM1_PWM_CHANNEL3_init(void);

#endif /* INC_CH32V00X_RVMSIS_H_ */
