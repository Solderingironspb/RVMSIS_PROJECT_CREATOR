/*
 * ch32v00x_RVMSIS.h
 *
 *  Created on: May 16, 2024
 *      Author: Solderingiron
 */

#ifndef INC_CH32V00X_RVMSIS_H_
#define INC_CH32V00X_RVMSIS_H_

#include "main.h"

#define __WEAK                                __attribute__((weak))
#define __INTERRUPTF                          __attribute__ ((interrupt("WCH-Interrupt-fast")))
#define __INTERRUPTM                          __attribute__((interrupt("machine")))

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


void RVMSIS_RCC_SystemClock_48MHz(void);
void RVMSIS_SysTick_Timer_init(void);
void Delay_ms(uint32_t Milliseconds);
void RVMSIS_GPIO_init(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Speed);
void RVMSIS_TIM1_init(void);
void RVMSIS_TIM1_PWM_CHANNEL1_init(void);
void RVMSIS_TIM1_PWM_CHANNEL2_init(void);
void RVMSIS_TIM2_init(void);
void RVMSIS_ADC_DMA_init(void);
void RVMSIS_I2C_Reset(void);
void RVMSIS_I2C1_Init(void);
bool RVMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms);
bool RVMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);
bool RVMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms);

#endif /* INC_CH32V00X_RVMSIS_H_ */
