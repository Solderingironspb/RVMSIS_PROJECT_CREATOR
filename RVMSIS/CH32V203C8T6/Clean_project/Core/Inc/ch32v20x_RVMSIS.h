/*
 * ch32v20x_RVMSIS.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Solderingiron
 */

#ifndef __CH32V20X_RVMSIS_H
#define __CH32V20X_RVMSIS_H

#include "main.h"
#include "ch32v20x.h"
#include "ch32v20x_rcc.h"
#include <stdbool.h>

#define __WEAK                                __attribute__((weak))
#define __INTERRUPTF                          __attribute__ ((interrupt("WCH-Interrupt-fast")))
#define __INTERRUPTM                          __attribute__((interrupt("machine")))

#define USART_MAX_LEN_RX_BUFFER                20


struct USART_name {
    uint8_t rx_buffer[USART_MAX_LEN_RX_BUFFER]; 
    uint16_t rx_counter; 
    uint16_t rx_len; 
};

//GPIO Configuration mode
enum {
    GPIO_GENERAL_PURPOSE_OUTPUT,
    GPIO_ALTERNATIVE_FUNCTION_OUTPUT,
    GPIO_INPUT
};
//GPIO Input/OUTPUT type
enum {
    GPIO_OUTPUT_PUSH_PULL,
    GPIO_OUTPUT_OPEN_DRAIN,
    GPIO_INPUT_ANALOG,
    GPIO_INPUT_FLOATING,
    GPIO_INPUT_PULL_DOWN,
    GPIO_INPUT_PULL_UP
};

//GPIO Maximum output speed
enum {
    GPIO_SPEED_RESERVED,
    GPIO_SPEED_10_MHZ,
    GPIO_SPEED_2_MHZ,
    GPIO_SPEED_50_MHZ
};

void RVMSIS_Debug_init(void);
void RVMSIS_RCC_SystemClock_144MHz(void);
void RVMSIS_SysTick_Timer_init(void);
void Delay_ms(uint32_t Milliseconds);
void RVMSIS_PC13_OUTPUT_Push_Pull_init(void);
void RVMSIS_Blink_PC13(uint32_t ms);
void RVMSIS_GPIO_init(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Speed); 
void RVMSIS_PA8_MCO_init(void);
void RVMSIS_EXTI0_init(void);
void RVMSIS_TIM3_init(void);
void RVMSIS_TIM3_PWM_CHANNEL1_init(void);
void RVMSIS_TIM3_PWM_CHANNEL2_init(void);
void RVMSIS_ADC_DMA_init(void);
void RVMSIS_USART1_Init(void);
void RVMSIS_USART2_Init(void);
void RVMSIS_USART3_Init(void);
bool RVMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms);
void RVMSIS_I2C_Reset(void);
void RVMSIS_I2C1_Init(void);
bool RVMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms);
bool RVMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
void RVMSIS_SPI1_init(void);
bool RVMSIS_SPI_Data_Transmit_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_SPI_Data_Transmit_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_SPI_Data_Receive_8BIT(SPI_TypeDef* SPI, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_SPI_Data_Receive_16BIT(SPI_TypeDef* SPI, uint16_t* data, uint16_t Size_data, uint32_t Timeout_ms);
void RVMSIS_FLASH_Unlock(void);
void RVMSIS_FLASH_Lock(void);
void RVMSIS_FLASH_Page_erase(uint16_t Adress);
void RVMSIS_FLASH_Page_write(uint32_t Adress, uint8_t *Data, uint16_t Size);
void RVMSIS_FLASH_Read_data(uint32_t Adress, uint8_t *Data, uint16_t Size);

#endif /* __CH32V20X_RVMSIS_H */
