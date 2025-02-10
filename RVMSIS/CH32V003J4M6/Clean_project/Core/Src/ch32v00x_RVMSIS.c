/*
 * ch32v00x_RVMSIS.c
 *
 *  Created on: May 16, 2024
 *      Author: Solderingiron
 */
#include "ch32v00x_RVMSIS.h"
/*============================== §¯§¡§³§´§²§°§«§¬§¡ RCC =======================================*/
/**
 ***************************************************************************************
 *  @breif §¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §®§¬ CH32V003J4M6 §ß§Ñ §é§Ñ§ã§ä§à§ä§å 48MHz §à§ä §Ó§ß§å§ä§â§Ö§ß§ß§Ö§Ô§à RC §Ô§Ö§ß§Ö§â§Ñ§ä§à§â§Ñ
 *  §£§ß§å§ä§â§Ö§ß§ß§Ú§Û RC §Ô§Ö§ß§Ö§â§Ñ§ä§à§â§Ñ §ß§Ñ 24 MHz
 *  ADC §ß§Ñ§ã§ä§â§à§Ö§ß §ß§Ñ 24MHz
 *  SYSCLK 48MHz
 *
 ***************************************************************************************
 */
void RVMSIS_RCC_SystemClock_48MHz(void) {
    SET_BIT(RCC->CTLR, RCC_HSION); //§©§Ñ§á§å§ã§ä§Ú§Þ §Ó§ß§å§ä§â§Ö§ß§ß§Ú§Û RC §Ô§Ö§ß§Ö§â§Ñ§ä§à§â §ß§Ñ 24 §®§¤§è
    while (READ_BIT(RCC->CTLR, RCC_HSIRDY) == 0);
    //§¥§à§Ø§Õ§Ö§Þ§ã§ñ §á§à§Õ§ß§ñ§ä§Ú§ñ §æ§Ý§Ñ§Ô§Ñ §à §Ô§à§ä§à§Ó§ß§à§ã§ä§Ú
    CLEAR_BIT(RCC->CTLR, RCC_HSEBYP);//§±§â§à§ã§ä§à §ã§Ò§â§à§ã§Ú§Þ §ï§ä§à§ä §Ò§Ú§ä §Ó 0(§·§à§ä§ñ §Ú§Ù§ß§Ñ§é§Ñ§Ý§î§ß§à §à§ß §Ú §ä§Ñ§Ü §Õ§à§Ý§Ø§Ö§ß §Ò§í§ä§î §Ó 0).
    CLEAR_BIT(RCC->CTLR, RCC_HSEON); //§£§ß§Ö§ê§ß§Ú§Û §Ü§Ó§Ñ§â§è§Ö§Ó§í§Û §â§Ö§Ù§à§ß§Ñ§ä§à§â §à§ä§ã§å§ä§ã§ä§Ó§å§Ö§ä.
    CLEAR_BIT(RCC->CTLR, RCC_CSSON); //§£§í§Ü§Ý§ð§é§Ú§Þ CSS
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b00 << RCC_SW_Pos); //§£§í§Ò§Ö§â§Ö§Þ HSI §Ó §Ü§Ñ§é§Ö§ã§ä§Ó§Ö System Clock(PLL §Ý§å§é§ê§Ö §á§à§Ü§Ñ §ß§Ö §Ó§í§Ò§Ú§â§Ñ§ä§î, §à§ß §å §ß§Ñ§ã §à§ä§Ü§Ý§ð§é§Ö§ß)
    CLEAR_BIT(RCC->CTLR, RCC_PLLON); //§£§í§Ü§Ý§ð§é§Ú§Þ PLL
    MODIFY_REG(RCC->CFGR0, RCC_HPRE, 0b00 << RCC_HPRE_Pos); //§ß§Ö §Õ§Ö§Ý§Ú§Þ §ß§Ú§é§Ö§Ô§à, §é§Ñ§ã§ä§à§ä§Ñ §Õ§à §Ó§Ü§Ý§ð§é§Ö§ß§Ú§ñ §Õ§à§Ý§Ø§ß§Ñ §Ò§í§ä§î 24§Þ§Ô§è
    //§¶§Ý§Ö§ê §Õ§à§Ý§Ø§Ö§ß §ã§Ñ§Þ §Õ§Ö§Ý§Ú§ä§î§ã§ñ §ß§Ñ 3
    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t) ((uint32_t) ~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t) FLASH_ACTLR_LATENCY_1;
    MODIFY_REG(RCC->CFGR0, RCC_ADCPRE, 0b00100 << RCC_ADCPRE_Pos);    //ADC Prescaler /6, §é§ä§à§Ò §Ò§í§Ý§à 12MHz, §ä.§Ü. §Þ§Ñ§Ü§ã§Ú§Þ§Ñ§Ý§î§ß§Ñ§ñ §é§Ñ§ã§ä§à§ä§Ñ §ä§å§ä 14 MHz
    CLEAR_BIT(RCC->CFGR0, RCC_PLLSRC); //HSI §ß§Ö §Õ§Ö§Ý§Ú§ä§ã§ñ §Ú §ß§Ö §à§ä§á§â§Ñ§Ó§Ý§ñ§Ö§ä§ã§ñ §Ó PLL.
    SET_BIT(RCC->CTLR, RCC_PLLON); //§©§Ñ§á§å§ã§ä§Ú§Þ PLL
    //§´.§Ü. PLL §å§Ø§Ö §Ù§Ñ§á§å§ë§Ö§ß, §Ó§í§Ò§Ö§â§Ö§Þ §Ö§Ô§à §Ó §Ü§Ñ§é§Ö§ã§ä§Ó§Ö System Clock:
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b10 << RCC_SW_Pos);//§£§í§Ò§Ö§â§Ö§Þ PLL §Ó §Ü§Ñ§é§Ö§ã§ä§Ó§Ö System Clock
    while (READ_BIT(RCC->CTLR, RCC_PLLRDY) == 0);
    //§¥§à§Ø§Ú§Õ§Ö§Þ§ã§ñ §á§à§Õ§ß§ñ§ä§Ú§ñ §æ§Ý§Ñ§Ô§Ñ §Ó§Ü§Ý§ð§é§Ö§ß§Ú§ñ PLL
    MODIFY_REG(RCC->CFGR0, RCC_SWS, 0b10 << RCC_SWS_Pos);
}

/*========================= §¯§¡§³§´§²§°§«§¬§¡ §³§ª§³§´§¦§®§¯§°§¤§° §´§¡§«§®§¦§²§¡ ==============================*/

/**
 ***************************************************************************************
 *  @breif §¯§Ñ§ã§ä§â§à§Û§Ü§Ñ SysTick §ß§Ñ §Þ§Ú§Ü§â§à§ã§Ö§Ü§å§ß§Õ§í
 *  §¯§Ñ §ï§ä§à§Þ §ä§Ñ§Û§Þ§Ö§â§Ö §Þ§í §ß§Ñ§ã§ä§â§à§Ú§Þ Delay §Ú §Ñ§ß§Ñ§Ý§à§Ô HAL_GetTick()
 ***************************************************************************************
 */
void RVMSIS_SysTick_Timer_init(void) {
    SysTick->CTLR &= ~(1 << 0); //§£§í§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Û§Þ§Ö§â §Õ§Ý§ñ §á§â§à§Ó§Ö§Õ§Ö§ß§Ú§ñ §ß§Ñ§ã§ä§â§à§Ö§Ü.
    SysTick->CTLR |= (1 << 1); //1: Enable counter interrupts.
    SysTick->CTLR &= ~(1 << 2); //0: HCLK for time base.48/8 = 6
    SysTick->CTLR |= (1 << 3); //1: Count up to the comparison value and start counting from 0 again
    SysTick->CMP = 5999; ////§¯§Ñ§ã§ä§â§à§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §ß§Ñ §é§Ñ§ã§ä§à§ä§å §Ó 1 §Ü§¤§è(§ä.§Ö. §ã§â§Ñ§Ò§à§ä§Ü§Ñ §Ò§å§Õ§Ö§ä §Ü§Ñ§Ø§Õ§å§ð §Þ§ã) 18000000 / 18000 = 1000§¤§è
    SysTick->CNT = 5999;
    NVIC_EnableIRQ(SysTicK_IRQn);
    NVIC_SetPriority(SysTicK_IRQn, 1);
    SysTick->CTLR |= (1 << 0); //§©§Ñ§á§å§ã§ä§Ú§Þ §ä§Ñ§Û§Þ§Ö§â.
}

/**
 ***************************************************************************************
 *  @breif §¯§Ñ§ã§ä§â§à§Û§Ü§Ñ Delay §Ú §Ñ§ß§Ñ§Ý§à§Ô HAL_GetTick()
 ***************************************************************************************
 */
volatile uint32_t SysTimer_ms = 0; //§±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ, §Ñ§ß§Ñ§Ý§à§Ô§Ú§é§ß§Ñ§ñ HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //§³§é§Ö§ä§é§Ú§Ü §Õ§Ý§ñ §æ§å§ß§Ü§è§Ú§Ú Delay_ms
volatile uint32_t Timeout_counter_ms = 0; //§±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ §Õ§Ý§ñ §ä§Ñ§Û§Þ§Ñ§å§ä§Ñ §æ§å§ß§Ü§è§Ú§Û

/**
 ******************************************************************************
 *  @breif §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §æ§Ý§Ñ§Ô§å CNTIF
 ******************************************************************************
 */
__WEAK void SysTick_Handler(void) {
    SysTick->SR &= ~(1 << 0);
    SysTimer_ms++;

    if (Delay_counter_ms) {
        Delay_counter_ms--;
    }
    if (Timeout_counter_ms) {
        Timeout_counter_ms--;
    }
}

/**
 ******************************************************************************
 *  @breif Delay_ms
 *  @param   uint32_t Milliseconds - §¥§Ý§Ú§ß§Ñ §Ù§Ñ§Õ§Ö§â§Ø§Ü§Ú §Ó §Þ§Ú§Ý§Ý§Ú§ã§Ö§Ü§å§ß§Õ§Ñ§ç
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
    Delay_counter_ms = Milliseconds;
    while (Delay_counter_ms != 0);
}

/*============================== §¯§¡§³§´§²§°§«§¬§¡ GPIO =======================================*/

//§³§Ý§å§Ø§Ö§Ò§ß§Ñ§ñ §æ§å§ß§Ü§è§Ú§ñ
static void RVMSIS_GPIO_MODE_Set(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, uint8_t Reg, uint8_t Data) {
    uint8_t Mode = 0;
    switch (Reg) {
    case (0):
        Mode = GPIO_Pin * 4;
        MODIFY_REG(GPIO->CFGLR, (0x3UL << Mode), Data << Mode);
        break;
    case (1):
        GPIO_Pin = GPIO_Pin - 8;
        Mode = GPIO_Pin * 4;
        MODIFY_REG(GPIO->CFGHR, (0x3UL << Mode), Data << Mode);
        break;
    }
}

//§³§Ý§å§Ø§Ö§Ò§ß§Ñ§ñ §æ§å§ß§Ü§è§Ú§ñ
static void RVMSIS_GPIO_SPEED_Set(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, uint8_t Speed) {
    uint8_t Reg = 0;
    if (GPIO_Pin < 8) {
        Reg = 0;
    } else {
        Reg = 1;
    }
    //MODE
    if (Speed == GPIO_SPEED_RESERVED) {
        RVMSIS_GPIO_MODE_Set(GPIO, GPIO_Pin, Reg, 0b00);
    } else if (Speed == GPIO_SPEED_10_MHZ) {
        RVMSIS_GPIO_MODE_Set(GPIO, GPIO_Pin, Reg, 0b01);
    } else if (Speed == GPIO_SPEED_2_MHZ) {
        RVMSIS_GPIO_MODE_Set(GPIO, GPIO_Pin, Reg, 0b10);
    } else if (Speed == GPIO_SPEED_50_MHZ) {
        RVMSIS_GPIO_MODE_Set(GPIO, GPIO_Pin, Reg, 0b11);
    }
}

//§³§Ý§å§Ø§Ö§Ò§ß§Ñ§ñ §æ§å§ß§Ü§è§Ú§ñ
static void RVMSIS_GPIO_CNF_Set(GPIO_TypeDef *GPIO, uint8_t Reg, uint8_t Mode, uint8_t* CNF_Pos) {
    switch (Reg) {
    case (0):
        MODIFY_REG(GPIO->CFGLR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
        break;
    case (1):
        MODIFY_REG(GPIO->CFGHR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
    }
}

//§³§Ý§å§Ø§Ö§Ò§ß§Ñ§ñ §æ§å§ß§Ü§è§Ú§ñ
static void RVMSIS_GPIO_Reg_Set(GPIO_TypeDef *GPIO, uint8_t* GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Reg, uint8_t* CNF_Pos) {
    switch (Configuration_mode) {
    case (GPIO_GENERAL_PURPOSE_OUTPUT):
        switch (Type) {
        case (GPIO_OUTPUT_PUSH_PULL):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b00, *(&CNF_Pos));
            break;
        case (GPIO_OUTPUT_OPEN_DRAIN):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b01, *(&CNF_Pos));
            break;
        }
        break;
    case (GPIO_ALTERNATIVE_FUNCTION_OUTPUT):
        switch (Type) {
        case (GPIO_OUTPUT_PUSH_PULL):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b10, *(&CNF_Pos));
            break;
        case (GPIO_OUTPUT_OPEN_DRAIN):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b11, *(&CNF_Pos));
            break;
        }
        break;
    case (GPIO_INPUT):
        switch (Type) {
        case (GPIO_INPUT_ANALOG):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b00, *(&CNF_Pos));
            break;
        case (GPIO_INPUT_FLOATING):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b01, *(&CNF_Pos));
            break;
        case (GPIO_INPUT_PULL_DOWN):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b10, *(&CNF_Pos));
            CLEAR_BIT(GPIO->OUTDR, (0x1UL << *GPIO_Pin));
            break;
        case (GPIO_INPUT_PULL_UP):
            RVMSIS_GPIO_CNF_Set(GPIO, Reg, 0b10, *(&CNF_Pos));
            SET_BIT(GPIO->OUTDR, (0x1UL << *GPIO_Pin));
            break;
        }
        break;
    }
}

/**
 ***************************************************************************************
 *  @breif §¢§í§ã§ä§â§Ñ§ñ §Ü§à§ß§æ§Ú§Ô§å§â§Ñ§è§Ú§ñ GPIO
 *  Reference Manual/§ã§Þ. §á.9.2 GPIO registers (§ã§ä§â. 171)
 *  §±§Ö§â§Ö§Õ §ß§Ñ§ã§ä§â§à§Û§Ü§à§Û (GPIOs and AFIOs) §ß§å§Ø§ß§à §Ó§Ü§Ý§ð§é§Ú§ä§î §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ.
 *  @param  *GPIO - §±§à§â§ä GPIO(A, B, C, D, E)
 *  @param  GPIO_Pin - §ß§à§Þ§Ö§â §á§Ú§ß§Ñ 0-15
 *  @param  Congiguration_mode: GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_ALTERNATIVE_FUNCTION_OUTPUT, GPIO_INPUT
 *  @param  Type: GPIO_OUTPUT_PUSH_PULL,
 *                GPIO_OUTPUT_OPEN_DRAIN,
 *                GPIO_INPUT_ANALOG,
 *                GPIO_INPUT_FLOATING,
 *                GPIO_INPUT_PULL_DOWN,
 *                GPIO_INPUT_PULL_UP
 *  @param  Speed: GPIO_SPEED_RESERVED,
 *                 GPIO_SPEED_10_MHZ,
 *                 GPIO_SPEED_2_MHZ,
 *                 GPIO_SPEED_50_MHZ
 ***************************************************************************************
 */

void RVMSIS_GPIO_init(GPIO_TypeDef *GPIO, uint8_t GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Speed) {
    uint8_t CNF_Pos = 0;
    if (GPIO == GPIOA) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §á§à§â§ä§Ñ §¡
    } else if (GPIO == GPIOC) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §á§à§â§ä§Ñ C
    } else if (GPIO == GPIOD) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOD); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §á§à§â§ä§Ñ D
    }

    RVMSIS_GPIO_SPEED_Set(GPIO, GPIO_Pin, Speed);

    if (GPIO_Pin < 8) {
        CNF_Pos = (GPIO_Pin * 4) + 2;
        RVMSIS_GPIO_Reg_Set(GPIO, (uint8_t*) &GPIO_Pin, Configuration_mode, Type, 0, &CNF_Pos);
    } else {
        GPIO_Pin = GPIO_Pin - 8;
        CNF_Pos = (GPIO_Pin * 4) + 2;
        RVMSIS_GPIO_Reg_Set(GPIO, (uint8_t*) &GPIO_Pin, Configuration_mode, Type, 1, &CNF_Pos);
    }
}

/*§´§Ñ§Û§Þ§Ö§â 1 §Õ§Ý§ñ §á§â§Ú§Þ§Ö§â§Ñ*/
void RVMSIS_TIM1_init(void) {
    /*§£§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §ä§Ñ§Û§Þ§Ö§â§Ñ (§ã§ä§â§Ñ§ß§Ú§è§Ñ 48)*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_TIM1); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §ä§Ñ§Û§Þ§Ö§â§Ñ 1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û

    CLEAR_BIT(TIM1->CTLR1, TIM_UDIS); //§¤§Ö§ß§Ö§â§Ú§â§à§Ó§Ñ§ä§î §ã§à§Ò§í§ä§Ú§Ö Update
    CLEAR_BIT(TIM1->CTLR1, TIM_URS); //§¤§Ö§ß§Ö§â§Ú§â§à§Ó§Ñ§ä§î §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö
    CLEAR_BIT(TIM1->CTLR1, TIM_OPM); //One pulse mode off(§³§é§Ö§ä§é§Ú§Ü §ß§Ö §à§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§ä§ã§ñ §á§â§Ú §à§Ò§ß§à§Ó§Ý§Ö§ß§Ú§Ú)
    CLEAR_BIT(TIM1->CTLR1, TIM_DIR); //§³§é§Ú§ä§Ñ§Ö§Þ §Ó§ß§Ú§Ù
    MODIFY_REG(TIM1->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos); //§£§í§â§Ñ§Ó§ß§Ú§Ó§Ñ§ß§Ú§Ö §á§à §Ü§â§Ñ§ð
    SET_BIT(TIM1->CTLR1, TIM_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM1->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos); //§±§â§Ö§Õ§Õ§Ö§Ý§Ö§ß§Ú§Ö §Ó§í§Ü§Ý§ð§é§Ö§ß§à

    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Û (§³§ä§â§Ñ§ß§Ú§è§Ñ 409)*/
    SET_BIT(TIM1->DMAINTENR, TIM_UIE); //Update interrupt enable

    TIM1->PSC = 200 - 1;
    TIM1->ATRLR = 1000 - 1;

    /*§¥§Ý§ñ §â§Ñ§Ò§à§ä§í §º§ª§®*/
    MODIFY_REG(TIM1->BDTR, TIM_LOCK, 0b00 << TIM_LOCK_Pos);
    SET_BIT(TIM1->BDTR, TIM_AOE);

    NVIC_EnableIRQ(TIM1_UP_IRQn); //§²§Ñ§Ù§â§Ö§ê§Ú§ä§î §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ §á§à §ä§Ñ§Û§Þ§Ö§â§å 3
    NVIC_SetPriority(TIM1_UP_IRQn, 0);
    SET_BIT(TIM1->CTLR1, TIM_CEN); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Û§Þ§Ö§â§Ñ
}

__WEAK void TIM1_UP_IRQHandler(void) {
    if (READ_BIT(TIM1->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM1->INTFR, TIM_UIF); //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ
    }
}

void RVMSIS_TIM1_PWM_CHANNEL1_init(void) {
    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ß§à§Ø§Ü§Ú PA1 §á§à§Õ §º§ª§®*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§£§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ §¡
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b10 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos);

    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ê§Ú§Þ(§¬§Ñ§ß§Ñ§Ý 1)*/
    MODIFY_REG(TIM1->CHCTLR1, TIM_CC1S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC1FE); //Fast mode disable
    SET_BIT(TIM1->CHCTLR1, TIM_OC1PE); //Preload enable
    MODIFY_REG(TIM1->CHCTLR1, TIM_OC1M, 0b110 << TIM_OC1M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC1CE); //OC1Ref is not affected by the ETRF input

    /*§©§Ñ§á§å§ã§Ü §º§ª§®*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM1->CCER, TIM_CC1E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM1->CCER, TIM_CC1P); //OC1 active high.

    TIM1->CH1CVR = 512;
}

void RVMSIS_TIM1_PWM_CHANNEL2_init(void) {
    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ß§à§Ø§Ü§Ú PA1 §á§à§Õ §º§ª§®*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§£§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ §¡
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b10 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos);

    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ê§Ú§Þ(§¬§Ñ§ß§Ñ§Ý 2)*/
    MODIFY_REG(TIM1->CHCTLR1, TIM_CC2S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC2FE); //Fast mode disable
    SET_BIT(TIM1->CHCTLR1, TIM_OC2PE); //Preload enable
    MODIFY_REG(TIM1->CHCTLR1, TIM_OC2M, 0b110 << TIM_OC2M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC2CE); //OC1Ref is not affected by the ETRF input

    /*§©§Ñ§á§å§ã§Ü §º§ª§®*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM1->CCER, TIM_CC2E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM1->CCER, TIM_CC2P); //OC1 active high.

    TIM1->CH2CVR = 512;
}

void RVMSIS_TIM2_init(void) {
    /*§£§Ü§Ý§ð§é§Ú§Þ §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §ä§Ñ§Û§Þ§Ö§â§Ñ (§ã§ä§â§Ñ§ß§Ú§è§Ñ 48)*/
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_TIM2); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §ä§Ñ§Û§Þ§Ö§â§Ñ 1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û

    CLEAR_BIT(TIM2->CTLR1, TIM_UDIS); //§¤§Ö§ß§Ö§â§Ú§â§à§Ó§Ñ§ä§î §ã§à§Ò§í§ä§Ú§Ö Update
    CLEAR_BIT(TIM2->CTLR1, TIM_URS); //§¤§Ö§ß§Ö§â§Ú§â§à§Ó§Ñ§ä§î §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö
    CLEAR_BIT(TIM2->CTLR1, TIM_OPM); //One pulse mode off(§³§é§Ö§ä§é§Ú§Ü §ß§Ö §à§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§ä§ã§ñ §á§â§Ú §à§Ò§ß§à§Ó§Ý§Ö§ß§Ú§Ú)
    CLEAR_BIT(TIM2->CTLR1, TIM_DIR); //§³§é§Ú§ä§Ñ§Ö§Þ §Ó§ß§Ú§Ù
    MODIFY_REG(TIM2->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos); //§£§í§â§Ñ§Ó§ß§Ú§Ó§Ñ§ß§Ú§Ö §á§à §Ü§â§Ñ§ð
    SET_BIT(TIM2->CTLR1, TIM_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM2->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos); //§±§â§Ö§Õ§Õ§Ö§Ý§Ö§ß§Ú§Ö §Ó§í§Ü§Ý§ð§é§Ö§ß§à

    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Û (§³§ä§â§Ñ§ß§Ú§è§Ñ 409)*/
    SET_BIT(TIM2->DMAINTENR, TIM_UIE); //Update interrupt enable

    TIM2->PSC = 48 - 1;
    TIM2->ATRLR = 1000 - 1;

    /*§¥§Ý§ñ §â§Ñ§Ò§à§ä§í §º§ª§®*/
    //MODIFY_REG(TIM1->BDTR, TIM_LOCK, 0b00 << TIM_LOCK_Pos);
    //SET_BIT(TIM1->BDTR, TIM_AOE);
    NVIC_EnableIRQ(TIM2_IRQn); //§²§Ñ§Ù§â§Ö§ê§Ú§ä§î §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ §á§à §ä§Ñ§Û§Þ§Ö§â§å 3
    NVIC_SetPriority(TIM2_IRQn, 2);
    SET_BIT(TIM2->CTLR1, TIM_CEN); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Û§Þ§Ö§â§Ñ
}

__WEAK void TIM2_IRQHandler(void) {
    if (READ_BIT(TIM2->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM2->INTFR, TIM_UIF); //§³§Ò§â§à§ã§Ú§Þ §æ§Ý§Ñ§Ô §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ
    }
}

/*================================= §¯§¡§³§´§²§°§«§¬§¡ ADC ============================================*/

/**
 ***************************************************************************************
 *  @breif Analog-to-digital converter (ADC)
 ***************************************************************************************
 */

volatile uint16_t ADC_RAW_Data[2] = { 0, }; //§®§Ñ§ã§ã§Ú§Ó, §Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ü§Ú§Õ§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö §ã §¡§¸§±

void RVMSIS_ADC_DMA_init(void) {
    //Chapter 11 Direct Memory Access Control (DMA)
    SET_BIT(RCC->AHBPCENR, RCC_AHBPeriph_DMA1); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ DMA1
    DMA1_Channel1->PADDR = (uint32_t) &(ADC1->RDATAR); //§©§Ñ§Õ§Ñ§Ö§Þ §Ñ§Õ§â§Ö§ã §á§Ö§â§Ú§æ§Ö§â§Ú§Û§ß§à§Ô§à §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
    DMA1_Channel1->MADDR = (uint32_t) ADC_RAW_Data; //§©§Ñ§Õ§Ñ§Ö§Þ §Ñ§Õ§â§Ö§ã §Ó §á§Ñ§Þ§ñ§ä§Ú, §Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ü§Ú§Õ§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö.
    DMA1_Channel1->CNTR = 1; //§¯§Ñ§ã§ä§â§à§Ú§Þ §Ü§à§Ý§Ú§é§Ö§ã§ä§Ó§à §Õ§Ñ§ß§ß§í§ç §Õ§Ý§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú. §±§à§ã§Ý§Ö §Ü§Ñ§Ø§Õ§à§Ô§à §á§Ö§â§Ú§æ§Ö§â§Ú§Û§ß§à§Ô§à §ã§à§Ò§í§ä§Ú§ñ §ï§ä§à §Ù§ß§Ñ§é§Ö§ß§Ú§Ö §Ò§å§Õ§Ö§ä §å§Þ§Ö§ß§î§ê§Ñ§ä§î§ã§ñ.
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PL, 0b00 << DMA_CFGR1_PL_Pos); //§©§Ñ§Õ§Ñ§Õ§Ú§Þ §á§â§Ú§à§â§Ú§ä§Ö§ä §Ü§Ñ§ß§Ñ§Ý§Ñ §ß§Ñ §Ó§í§ã§à§Ü§Ú§Û
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_DIR); //§¹§ä§Ö§ß§Ú§Ö §Ò§å§Õ§Ö§Þ §à§ã§å§ë§Ö§ã§ä§Ó§Ý§ñ§ä§î §ã §á§Ö§â§Ú§æ§Ö§â§Ú§Ú
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_CIRC); //§¯§Ñ§ã§ä§â§à§Ú§Þ DMA §Ó Circular mode
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PSIZE, 0b01 << DMA_CFGR1_PSIZE_Pos); //§²§Ñ§Ù§Þ§Ö§â §Õ§Ñ§ß§ß§í§ç §á§Ö§â§Ú§æ§Ö§â§Ú§Û§ß§à§Ô§à §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ 16 §Ò§Ú§ä
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_MSIZE, 0b01 << DMA_CFGR1_MSIZE_Pos); //§²§Ñ§Ù§Þ§Ö§â §Õ§Ñ§ß§ß§í§ç §Ó §á§Ñ§Þ§ñ§ä§Ú 16 §Ò§Ú§ä
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TCIE); //§£§Ü§Ý§ð§é§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §á§à§Ý§ß§à§Û §á§Ö§â§Ö§Õ§Ñ§é§Ö
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_HTIE); //§°§ä§Ü§Ý§ð§é§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §á§à§Ý§à§Ó§Ú§ß§ß§à§Û §á§Ö§â§Ö§Õ§Ñ§é§Ö
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TEIE); //§£§Ü§Ý§ð§é§Ú§Þ §á§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §à§ê§Ú§Ò§Ü§Ö §á§Ö§â§Ö§Õ§Ñ§é§Ú.
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_MINC); //§£§Ü§Ý§ð§é§Ú§Þ §Ú§ß§Ü§â§Ö§Þ§Ö§ß§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§Ñ§Þ§ñ§ä§Ú
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_EN); //DMA ON
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_ADC1); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ ADC1.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //§£§Ü§Ý§ð§é§Ö§ß§Ú§Ö §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §á§à§â§ä§Ñ §¡.

    /*§¯§Ñ§ã§ä§â§à§Û§Ü§Ñ §ß§à§Ø§Ü§Ú PA1 §Ú PA2 §ß§Ñ §Ñ§ß§Ñ§Ý§à§Ô§à§Ó§í§Û §Ó§ç§à§Õ*/
    /*Pin PA1 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b00 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b00 << GPIO_CFGLR_MODE1_Pos);

    /*Pin PA2 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF2, 0b00 << GPIO_CFGLR_CNF2_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE2, 0b00 << GPIO_CFGLR_MODE2_Pos);

    //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §¡§¸§±: §â§Ö§Ô§å§Ý§ñ§â§ß§í§Ö §Ü§Ñ§ß§Ñ§Ý§í (§Ó§Ü§Ý/§Ó§í§Ü§Ý)
    CLEAR_BIT(ADC1->CTLR1, ADC_EOCIE);//EOC interrupt enabled/disabled. An interrupt is generated when the EOC bit is set

    //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §¡§¸§±: analog watchdog (§Ó§Ü§Ý/§Ó§í§Ü§Ý)
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDIE);//Analog watchdog interrupt disabled

    //§±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö §á§à §¡§¸§±: §Ú§ß§Ø§Ö§Ü§ä§Ú§â§à§Ó§Ñ§ß§ß§í§Ö §Ü§Ñ§ß§Ñ§Ý§í (§Ó§Ü§Ý/§Ó§í§Ü§Ý)
    CLEAR_BIT(ADC1->CTLR1, ADC_JEOCIE);//JEOC interrupt disabled

    SET_BIT(ADC1->CTLR1, ADC_SCAN); //Scan mode enabled

    /* §±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö:
     * §±§â§Ö§â§í§Ó§Ñ§ß§Ú§Ö EOC §Ú§Ý§Ú JEOC §Ô§Ö§ß§Ö§â§Ú§â§å§Ö§ä§ã§ñ §ä§à§Ý§î§Ü§à §Ó §Ü§à§ß§è§Ö §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§ñ §á§à§ã§Ý§Ö§Õ§ß§Ö§Ô§à §Ü§Ñ§ß§Ñ§Ý§Ñ,
     * §Ö§ã§Ý§Ú §å§ã§ä§Ñ§ß§à§Ó§Ý§Ö§ß §ã§à§à§ä§Ó§Ö§ä§ã§ä§Ó§å§ð§ë§Ú§Û §Ò§Ú§ä EOCIE §Ú§Ý§Ú JEOCIE.*/

    CLEAR_BIT(ADC1->CTLR1, ADC_AWDSGL); //Analog watchdog enabled on all channels
    CLEAR_BIT(ADC1->CTLR1, ADC_JAUTO); //Automatic injected group conversion disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_DISCEN); //Discontinuous mode on regular channels disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_JDISCEN); //Discontinuous mode on injected channels disabled
    //MODIFY_REG(ADC1->CTLR1, ADC_DUALMOD, 0b0110 << ADC_DUALMOD_Pos); //0110: Regular simultaneous mode only
    CLEAR_BIT(ADC1->CTLR1, ADC_JAWDEN);//Analog watchdog disabled on injected channels
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDEN); //Analog watchdog disabled on regular channels

    //Control register 2 CTLR2
    SET_BIT(ADC1->CTLR2, ADC_ADON);//§©§Ñ§á§å§ã§ä§Ú§ä§î §¡§¸§±

    /* §±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö:
     * §¦§ã§Ý§Ú §Ó §ï§ä§à§ä §Ø§Ö §Þ§à§Þ§Ö§ß§ä §Ú§Ù§Þ§Ö§ß§ñ§Ö§ä§ã§ñ §Ü§Ñ§Ü§à§Û-§Ý§Ú§Ò§à §Õ§â§å§Ô§à§Û §Ò§Ú§ä §Ó §ï§ä§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö,
     * §Ü§â§à§Þ§Ö ADON, §ä§à §Ü§à§ß§Ó§Ö§â§ã§Ú§ñ §ß§Ö §Ù§Ñ§á§å§ã§Ü§Ñ§Ö§ä§ã§ñ.
     * §¿§ä§à §ã§Õ§Ö§Ý§Ñ§ß§à §Õ§Ý§ñ §á§â§Ö§Õ§à§ä§Ó§â§Ñ§ë§Ö§ß§Ú§ñ §à§ê§Ú§Ò§à§é§ß§à§Ô§à §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§ñ.*/

    SET_BIT(ADC1->CTLR2, ADC_CONT); //Continuous conversion mode(§ß§Ö§á§â§Ö§â§í§Ó§ß§í§Ö §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§ñ)
    SET_BIT(ADC1->CTLR2, ADC_CAL); //Enable calibration
    /*§±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö:
     * §¿§ä§à§ä §Ò§Ú§ä §å§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§à§Û §Õ§Ý§ñ §Ù§Ñ§á§å§ã§Ü§Ñ §Ü§Ñ§Ý§Ú§Ò§â§à§Ó§Ü§Ú.
     * §°§ß §ã§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§ä§ã§ñ §Ñ§á§á§Ñ§â§Ñ§ä§ß§à §á§à§ã§Ý§Ö §Ù§Ñ§Ó§Ö§â§ê§Ö§ß§Ú§ñ §Ü§Ñ§Ý§Ú§Ò§â§à§Ó§Ü§Ú.*/

    while (READ_BIT(ADC1->CTLR2, ADC_CAL));
    //§±§à§Õ§à§Ø§Õ§Ö§Þ §à§Ü§à§ß§é§Ñ§ß§Ú§ñ §Ü§Ñ§Ý§Ú§Ò§â§à§Ó§Ü§Ú
    //Delay_ms(10);

    SET_BIT(ADC1->CTLR2, ADC_DMA);//DMA §Ó§Ü§Ý§ð§é§Ö§ß
    CLEAR_BIT(ADC1->CTLR2, ADC_ALIGN); //§£§í§â§Ñ§Ó§ß§Ú§Ó§Ñ§ß§Ú§Ö §á§à §á§â§Ñ§Ó§à§Þ§å §Ü§â§Ñ§ð
    MODIFY_REG(ADC1->CTLR2, ADC_EXTSEL, 0b111 << ADC_EXTSEL_Pos); //§©§Ñ§á§å§ã§Ü§Ñ§ä§î §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§Ö §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à
    CLEAR_BIT(ADC1->CTLR2, ADC_EXTTRIG); //Conversion on external event disabled
    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //§¯§Ñ§é§Ñ§ä§î §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§Ö
    //SET_BIT(ADC1->CTLR2, ADC_TSVREFE);//Temperature sensor and VREFINT channel enabled

    // 12.3.5 ADCx Sample Time Configuration Register 2 (ADCx_SAMPTR2) (x=1/2)
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP1, 0b111 << ADC_SMP1_Pos);//239.5 cycles
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP0, 0b111 << ADC_SMP0_Pos);//239.5 cycles
    //MODIFY_REG(ADC1->SAMPTR2, ADC_SMP2, 0b111 << ADC_SMP2_Pos);//239.5 cycles
    //MODIFY_REG(ADC1->SAMPTR2, ADC_SMP1, 0b111 << ADC_SMP1_Pos); //239.5 cycles
    //MODIFY_REG(ADC1->SAMPTR1, ADC_SMP17, 0b111 << ADC_SMP17_Pos); //239.5 cycles

    //12.3.9 ADCx Regular Channel Sequence Register1 (ADCx_RSQR1) (x=1/2)
    MODIFY_REG(ADC1->RSQR1, ADC_L, 0b0000 << ADC_L_Pos);//1 §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§Ö

    //12.3.11 ADCx Regular Channel Sequence Register 3 (ADCx_RSQR3) (x=1/2)
    MODIFY_REG(ADC1->RSQR3, ADC_SQ1, 0 << ADC_SQ1_Pos);
    //MODIFY_REG(ADC1->RSQR3, ADC_SQ2, 0 << ADC_SQ2_Pos);
    //MODIFY_REG(ADC1->RSQR3, ADC_SQ2, 1 << ADC_SQ2_Pos);
    //NVIC_EnableIRQ(ADC1_2_IRQn); //§²§Ñ§Ù§â§Ö§ê§Ú§ä§î §á§â§Ö§â§í§Ó§Ñ§ß§Ú§ñ §á§à §¡§¸§±

    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //§¯§Ñ§é§Ñ§ä§î §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§Ö. §¯§Ö §ß§å§Ø§ß§à §Ù§Ñ§á§å§ã§Ü§Ñ§ä§î, §Ö§ã§Ý§Ú Continuous conversion mode(§ß§Ö§á§â§Ö§â§í§Ó§ß§í§Ö §á§â§Ö§à§Ò§â§Ñ§Ù§à§Ó§Ñ§ß§Ú§ñ) §Ó§Ü§Ý§ð§é§Ö§ß

}

__WEAK void ADC1_2_IRQHandler(void) {
    if (READ_BIT(ADC1->STATR, ADC_EOC)) {
        ADC1->IDATAR1; //§¹§Ú§ä§Ñ§Ö§Þ §Ü§Ñ§ß§Ñ§Ý, §é§ä§à§Ò §ã§Ò§â§à§ã§Ú§ä§î §æ§Ý§Ñ§Ô
    }

}
__WEAK void DMA1_Channel1_IRQHandler(void) {
    if (READ_BIT(DMA1->INTFR, DMA_TCIF1)) {
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //§³§Ò§â§à§ã§Ú§Þ §Ô§Ý§à§Ò§Ñ§Ý§î§ß§í§Û §æ§Ý§Ñ§Ô.
        /*§©§Õ§Ö§ã§î §Þ§à§Ø§ß§à §á§Ú§ã§Ñ§ä§î §Ü§à§Õ*/

    } else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
        /*§©§Õ§Ö§ã§î §Þ§à§Ø§ß§à §ã§Õ§Ö§Ý§Ñ§ä§î §Ü§Ñ§Ü§à§Û-§ä§à §à§Ò§â§Ñ§Ò§à§ä§é§Ú§Ü §à§ê§Ú§Ò§à§Ü*/
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //§³§Ò§â§à§ã§Ú§Þ §Ô§Ý§à§Ò§Ñ§Ý§î§ß§í§Û §æ§Ý§Ñ§Ô.
    }
}



/*================================= §¯§¡§³§´§²§°§«§¬§¡ I2C ============================================*/

/**
 ***************************************************************************************
 *  @breif Inter-integrated circuit (I2C) interface
 ***************************************************************************************
 */

void RVMSIS_I2C_Reset(void) {
    //§³§Ò§â§à§ã §ß§Ñ§ã§ä§â§à§Ö§Ü I2C
    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    SET_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST) == 0);
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST));
    /* §±§â§Ú§Þ§Ö§é§Ñ§ß§Ú§Ö: §¿§ä§à§ä §Ò§Ú§ä §Þ§à§Ø§ß§à §Ú§ã§á§à§Ý§î§Ù§à§Ó§Ñ§ä§î §Õ§Ý§ñ §á§à§Ó§ä§à§â§ß§à§Û §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§Ú
     * §á§Ö§â§Ú§æ§Ö§â§Ú§Û§ß§à§Ô§à §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ §á§à§ã§Ý§Ö §à§ê§Ú§Ò§Ü§Ú §Ú§Ý§Ú §Ù§Ñ§Ò§Ý§à§Ü§Ú§â§à§Ó§Ñ§ß§ß§à§Ô§à §ã§à§ã§ä§à§ñ§ß§Ú§ñ.
     * §¯§Ñ§á§â§Ú§Þ§Ö§â, §Ö§ã§Ý§Ú §Ò§Ú§ä BUSY §å§ã§ä§Ñ§ß§à§Ó§Ý§Ö§ß §Ú §à§ã§ä§Ñ§Ö§ä§ã§ñ §Ù§Ñ§Ò§Ý§à§Ü§Ú§â§à§Ó§Ñ§ß§ß§í§Þ §Ú§Ù-§Ù§Ñ §ã§Ò§à§ñ §ß§Ñ §ê§Ú§ß§Ö,
     * §Ò§Ú§ä SWRST §Þ§à§Ø§ß§à §Ú§ã§á§à§Ý§î§Ù§à§Ó§Ñ§ä§î §Õ§Ý§ñ §Ó§í§ç§à§Õ§Ñ §Ú§Ù §ï§ä§à§Ô§à §ã§à§ã§ä§à§ñ§ß§Ú§ñ.*/
}

/**
 *************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§Ú §ê§Ú§ß§í I2C1. Sm.
 *************************************************************************************
 */

void RVMSIS_I2C1_Init(void) {
    //§¯§Ñ§ã§ä§â§à§Û§Ü§Ú §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§Ö §á§à§â§ä§Ñ C
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ §Ñ§Ý§î§ä§Ö§â§ß§Ñ§ä§Ú§Ó§ß§í§ç §æ§å§ß§Ü§è§Ú§Û
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_I2C1); //§©§Ñ§á§å§ã§Ü §ä§Ñ§Ü§ä§Ú§â§à§Ó§Ñ§ß§Ú§ñ I2C1
    //SET_BIT(AFIO->PCFR1,AFIO_PCFR1_I2C1_REMAP); //§£§Ü§Ý§ð§é§Ú§Þ §â§Ö§Þ§Ñ§á

    //§¯§Ñ§ã§ä§â§à§Û§Ü§Ú §ß§à§Ø§Ö§Ü SDA §Ú SCL
    //PC1 SDA (I2C Data I/O) Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_CNF1, 0b11 << GPIO_CFGLR_CNF1_Pos);//Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos); //Maximum output speed 50 MHz
    //PC2 SCL (I2C clock) Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_CNF2, 0b11 << GPIO_CFGLR_CNF2_Pos);//Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_MODE2, 0b11 << GPIO_CFGLR_MODE2_Pos); //Maximum output speed 50 MHz

    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    RVMSIS_I2C_Reset();

    /*§¿§ä§à §Ó§ã§Ö §Õ§Ý§ñ §Ú§ß§Ú§ä§Ñ §ß§Ö §ß§å§Ø§ß§à. §±§à§ã§Ý§Ö §ã§Ò§â§à§ã§Ñ §Ú§ä§Ñ§Ü §Ò§å§Õ§Ö§ä §Ó 0. */
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ALERT); //Releases SMBA pin high.Alert Response Address Header followed by NACK
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_PEC); //No PEC transfer
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ACK); //No acknowledge returned
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_STOP); //No Stop generation
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_START); //No Start generation
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_NOSTRETCH); //Clock stretching enabled
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ENGC); //General call disabled. Address 00h is NACKed.
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ENPEC); //PEC calculation disabled
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ENARP); //ARP disable
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SMBTYPE); //SMBus Device
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SMBUS); //I2C mode

    //19.12.2 I2C Control Register 2 (I2Cx_CTLR2) (x=1/2)
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_LAST);//Next DMA EOT is not the last transfer
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_DMAEN); //DMA requests disabled
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_ITBUFEN); //TxE = 1 or RxNE = 1 does not generate any interrupt.
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_ITEVTEN); //Event interrupt disabled
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_ITERREN); //Error interrupt disabled
    MODIFY_REG(I2C1->CTLR2, I2C_CTLR2_FREQ, 36 << I2C_CTLR2_FREQ_Pos); //f PCLK1 = 36 §®§Ô§è

    //19.12.3 I2C Address Register 1 (I2Cx_OADDR1) (x=1/2)
    I2C1->OADDR1 = 0;
    //19.12.4 I2C Address Register2 (I2Cx_OADDR2) (x=1/2)
    I2C1->OADDR2 = 0;

    //19.12.8 I2C Clock Register (I2Cx_CKCFGR) (x=1/2)
    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS);//Standard mode I2C
    //SET_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS); //Fast mode I2C

    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_DUTY);//Fm mode tlow/thigh = 2
    //SET_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 16/9 (see CCR)

    //§²§Ñ§ã§é§Ö§ä CCR. §³§Þ§à§ä§â§Ú §á§â§Ú§Þ§Ö§â§í §â§Ñ§ã§é§Ö§ä§Ñ
    MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 180 << I2C_CKCFGR_CCR_Pos);//§Õ§Ý§ñ Sm mode
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 30 << I2C_CKCFGR_CCR_Pos); //§Õ§Ý§ñ Fm mode. DUTY 0.
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 4 << I2C_CKCFGR_CCR_Pos); //§Õ§Ý§ñ Fm mode. DUTY 1.

    //19.12.9 I2C Rise Time Register (I2Cx_RTR) (x=1/2)
    //MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 37 << I2C_RTR_TRISE_Pos);//§Õ§Ý§ñ Sm mode
    //MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 12 << I2C_RTR_TRISE_Pos); //§Õ§Ý§ñ Fm mode

    SET_BIT(I2C1->CTLR1, I2C_CTLR1_PE); //I2C1 enable
}

/**
 *************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §ã§Ü§Ñ§ß§Ú§â§à§Ó§Ñ§ß§Ú§ñ §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ §á§à §Ù§Ñ§Õ§Ñ§ß§ß§à§Þ§å 7-§Ò§Ú§ä§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã true - §Ö§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §á§à §Ù§Ñ§Õ§Ñ§ß§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î,
 *           false - §Ö§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §á§à §Ù§Ñ§Õ§Ñ§ß§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å §ß§Ö §à§ä§Ó§Ö§é§Ñ§Ö§ä
 *************************************************************************************
 */
bool RVMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§°§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ §ã§Ú§Ô§ß§Ñ§Ý START

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Õ§Ñ§ß§ß§í§ç §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //§¡§Õ§â§Ö§ã + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;
        return true;
    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú §Õ§Ñ§ß§ß§í§ç §á§à I2C
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ü§à§ä§à§â§í§Ö §Ò§å§Õ§Ö§Þ §à§ä§á§â§Ñ§Ó§Ý§ñ§ä§î
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§Þ §à§ä§á§â§Ñ§Ó§Ý§ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §à§ä§á§â§Ñ§Ó§Ü§Ú §Õ§Ñ§ß§ß§í§ç. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //§¡§Õ§â§Ö§ã + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*§°§ä§á§â§Ñ§Ó§Ú§Þ §Õ§Ñ§ß§ß§í§Ö*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //§©§Ñ§á§Ú§ã§î §Ò§Ñ§Û§ä§Ñ
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Õ§Ñ§ß§ß§í§Ö §Ù§Ñ§Ô§â§å§Ù§ñ§ä§ã§ñ §Ó §â§Ö§Ô§Ú§ã§ä§â §ã§Õ§Ó§Ú§Ô§Ñ.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ

        return true;

    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §á§â§Ú§Ö§Þ§Ñ §Õ§Ñ§ß§ß§í§ç §á§à I2C
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @param  *data - §¬§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §á§â§Ú§ß§ñ§ä§í§Ö §Õ§Ñ§ß§ß§í§Ö
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§Þ §á§â§Ú§ß§Ú§Þ§Ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §á§â§Ú§Ö§Þ§Ñ §Õ§Ñ§ß§ß§í§ç. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1 | 1); //§¡§Õ§â§Ö§ã + §Ü§à§Þ§Ñ§ß§Õ§Ñ Read

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*§±§â§à§é§ä§Ö§Þ §Õ§Ñ§ß§ß§í§Ö*/
        for (uint16_t i = 0; i < Size_data; i++) {
            if (i < Size_data - 1) {
                SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //§¦§ã§Ý§Ú §Þ§í §ç§à§ä§Ú§Þ §á§â§Ú§ß§ñ§ä§î §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §Ò§Ñ§Û§ä, §ä§à §à§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ ACK

                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //§°§Ø§Ú§Õ§Ñ§Ö§Þ, §á§à§Ü§Ñ §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö §á§à§ñ§Ó§ñ§ä§ã§ñ §Õ§Ñ§ß§ß§í§Ö
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }

                *(data + i) = I2C->DATAR; //§¹§ä§Ö§ß§Ú§Ö §Ò§Ñ§Û§ä§Ñ
            } else {
                CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //§¦§ã§Ý§Ú §Þ§í §Ù§ß§Ñ§Ö§Þ, §é§ä§à §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §á§â§Ú§ß§ñ§ä§í§Û §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§ä §á§à§ã§Ý§Ö§Õ§ß§Ú§Þ, §ä§à §à§ä§á§â§Ñ§Ó§Ú§Þ NACK

                SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //§°§Ø§Ú§Õ§Ñ§Ö§Þ, §á§à§Ü§Ñ §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö §á§à§ñ§Ó§ñ§ä§ã§ñ §Õ§Ñ§ß§ß§í§Ö
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }
                *(data + i) = I2C->DATAR; //§¹§ä§Ö§ß§Ú§Ö §Ò§Ñ§Û§ä§Ñ
            }
        }
        return true;

    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
        return false;
    }

}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §Ù§Ñ§á§Ú§ã§Ú §Ó §á§Ñ§Þ§ñ§ä§î §á§à §å§Ü§Ñ§Ù§Ñ§ß§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @param  Adress_data - §¡§Õ§â§Ö§ã §Ó §á§Ñ§Þ§ñ§ä§Ú, §Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö
 *  @param  Size_adress - §²§Ñ§Ù§Þ§Ö§â §Ñ§Õ§â§Ö§ã§Ñ §Ó §Ò§Ñ§Û§ä§Ñ§ç. §±§â§Ú§Þ§Ö§â: 1 - 8 §Ò§Ú§ä§ß§í§Û §Ñ§Õ§â§Ö§ã. 2 - 16 §Ò§Ú§ä§ß§í§Û §Ñ§Õ§â§Ö§ã.
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ü§à§ä§à§â§í§Ö §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §Ù§Ñ§á§Ú§ã§Ú. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //§¡§Õ§â§Ö§ã + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*§°§ä§á§â§Ñ§Ó§Ú§Þ §Ñ§Õ§â§Ö§ã §á§Ñ§Þ§ñ§ä§Ú*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //§©§Ñ§á§Ú§ã§î §Ò§Ñ§Û§ä§Ñ
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Õ§Ñ§ß§ß§í§Ö §Ù§Ñ§Ô§â§å§Ù§ñ§ä§ã§ñ §Ó §â§Ö§Ô§Ú§ã§ä§â §ã§Õ§Ó§Ú§Ô§Ñ.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
                    return false;
                }
            }
        }

        /*§¢§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö §Ó §ñ§é§Ö§Û§Ü§å §á§Ñ§Þ§ñ§ä§Ú, §ß§Ñ§é§Ú§ß§Ñ§ñ §ã §å§Ü§Ñ§Ù§Ñ§ß§ß§à§Ô§à §Ñ§Õ§â§Ö§ã§Ñ*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //§©§Ñ§á§Ú§ã§î §Ò§Ñ§Û§ä§Ñ
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Õ§Ñ§ß§ß§í§Ö §Ù§Ñ§Ô§â§å§Ù§ñ§ä§ã§ñ §Ó §â§Ö§Ô§Ú§ã§ä§â §ã§Õ§Ó§Ú§Ô§Ñ.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ

        return true;

    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif §¶§å§ß§Ü§è§Ú§ñ §é§ä§Ö§ß§Ú§ñ §Ú§Ù §á§Ñ§Þ§ñ§ä§Ú §á§à §å§Ü§Ñ§Ù§Ñ§ß§ß§à§Þ§å §Ñ§Õ§â§Ö§ã§å
 *  @param  *I2C - §ê§Ú§ß§Ñ I2C
 *  @param  Adress_Device - §¡§Õ§â§Ö§ã §å§ã§ä§â§à§Û§ã§ä§Ó§Ñ
 *  @param  Adress_data - §¡§Õ§â§Ö§ã §Ó §á§Ñ§Þ§ñ§ä§Ú, §à§ä§Ü§å§Õ§Ñ §Ò§å§Õ§Ö§Þ §ã§é§Ú§ä§í§Ó§Ñ§ä§î §Õ§Ñ§ß§ß§í§Ö
 *  @param  Size_adress - §²§Ñ§Ù§Þ§Ö§â §Ñ§Õ§â§Ö§ã§Ñ §Ó §Ò§Ñ§Û§ä§Ñ§ç. §±§â§Ú§Þ§Ö§â: 1 - 8 §Ò§Ú§ä§ß§í§Û §Ñ§Õ§â§Ö§ã. 2 - 16 §Ò§Ú§ä§ß§í§Û §Ñ§Õ§â§Ö§ã.
 *  @param  *data - §¥§Ñ§ß§ß§í§Ö, §Ó §Ü§à§ä§à§â§í§Ö §Ò§å§Õ§Ö§Þ §Ù§Ñ§á§Ú§ã§í§Ó§Ñ§ä§î §ã§é§Ú§ä§Ñ§ß§ß§å§ð §Ú§ß§æ§à§â§Þ§Ñ§è§Ú§ð.
 *  @param  Size_data - §²§Ñ§Ù§Þ§Ö§â, §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§Þ §ã§é§Ú§ä§í§Ó§Ñ§ä§î.
 *  @retval  §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä §ã§ä§Ñ§ä§å§ã §ã§é§Ú§ä§í§Ó§Ñ§ß§Ú§ñ. True - §µ§ã§á§Ö§ê§ß§à. False - §°§ê§Ú§Ò§Ü§Ñ.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //§¦§ã§Ý§Ú §ê§Ú§ß§Ñ §Ù§Ñ§ß§ñ§ä§Ñ

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //§¦§ã§Ý§Ú §Ý§Ú§ß§Ú§ñ §ß§Ñ §ã§Ñ§Þ§à§Þ §Õ§Ö§Ý§Ö §ã§Ó§à§Ò§à§Õ§ß§Ñ, §Ñ BUSY §Ó§Ú§ã§Ú§ä
            RVMSIS_I2C_Reset(); //§â§Ö§ã§Ö§ä
            RVMSIS_I2C1_Init(); //§á§à§Ó§ä§à§â§ß§Ñ§ñ §Ú§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ñ§è§Ú§ñ
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //§¦§ã§Ý§Ú §ã§ä§à§Ú§ä §ã§ä§Ñ§ä§å§ã, §é§ä§à §Þ§í §Ó §Þ§Ñ§ã§ä§Ö§â§Ö
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ä§á§â§Ñ§Ó§Ú§Þ §ã§Ú§Ô§ß§Ñ§Ý STOP
        }

        if (I2C->CTLR1 != 1) {
            //§¦§ã§Ý§Ú §Ó CR1 §é§ä§à-§ä§à §Ý§Ú§ê§ß§Ö§Ö, §ä§à §á§Ö§â§Ö§Ù§Ñ§Ô§â§å§Ù§Ú§Þ I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------§±§â§à§Ó§Ö§â§Ü§Ñ §Ù§Ñ§ß§ñ§ä§à§ã§ä§Ú §ê§Ú§ß§í-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //§¢§Ú§ä ACK §å§á§â§Ñ§Ó§Ý§ñ§Ö§ä (N)ACK §ä§Ö§Ü§å§ë§Ö§Ô§à §Ò§Ñ§Û§ä§Ñ, §á§â§Ú§ß§Ú§Þ§Ñ§Ö§Þ§à§Ô§à §Ó §ã§Õ§Ó§Ú§Ô§à§Ó§à§Þ §â§Ö§Ô§Ú§ã§ä§â§Ö.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //§£§¯§ª§®§¡§¯§ª§¦!
    /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //§¡§Õ§â§Ö§ã + §Ü§à§Þ§Ñ§ß§Õ§Ñ Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
        /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*§°§ä§á§â§Ñ§Ó§Ú§Þ §Ñ§Õ§â§Ö§ã §á§Ñ§Þ§ñ§ä§Ú*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //§©§Ñ§á§Ú§ã§î §Ò§Ñ§Û§ä§Ñ
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Õ§Ñ§ß§ß§í§Ö §Ù§Ñ§Ô§â§å§Ù§ñ§ä§ã§ñ §Ó §â§Ö§Ô§Ú§ã§ä§â §ã§Õ§Ó§Ú§Ô§Ñ.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
                    return false;
                }
            }
        }

        //§±§à§Ó§ä§à§â§ß§í§Û §ã§ä§Ñ§â§ä
        SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //§³§ä§Ñ§â§ä§å§Ö§Þ.

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
            //§°§Ø§Ú§Õ§Ñ§Ö§Þ §Õ§à §Þ§à§Þ§Ö§ß§ä§Ñ, §á§à§Ü§Ñ §ß§Ö §ã§â§Ñ§Ò§à§ä§Ñ§Ö§ä Start condition generated

            if (!Timeout_counter_ms) {
                return false;
            }

        }
        //§£§¯§ª§®§¡§¯§ª§¦!
        /* §¢§Ú§ä I2C_SR1_SB §à§é§Ú§ë§Ñ§Ö§ä§ã§ñ §á§â§à§Ô§â§Ñ§Þ§Þ§ß§à §á§å§ä§Ö§Þ §é§ä§Ö§ß§Ú§ñ §â§Ö§Ô§Ú§ã§ä§â§Ñ SR1 §ã §á§à§ã§Ý§Ö§Õ§å§ð§ë§Ö§Û §Ù§Ñ§á§Ú§ã§î§ð §Ó §â§Ö§Ô§Ú§ã§ä§â DR §Ú§Ý§Ú §Ü§à§Ô§Õ§Ñ PE=0*/
        I2C->STAR1;
        I2C->DATAR = (Adress_Device << 1 | 1); //§¡§Õ§â§Ö§ã + §Ü§à§Þ§Ñ§ß§Õ§Ñ Read

        Timeout_counter_ms = Timeout_ms;
        while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
            //§¨§Õ§Ö§Þ, §á§à§Ü§Ñ §Ñ§Õ§â§Ö§ã §à§ä§Ù§à§Ó§Ö§ä§ã§ñ

            if (!Timeout_counter_ms) {
                return false;
            }

        }

        if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
            //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §ã§Ò§â§à§ã§Ú§Þ §Ò§Ú§ä ADDR
            /*§³§Ò§â§à§ã §Ò§Ú§ä§Ñ ADDR §á§â§à§Ú§Ù§Ó§à§Õ§Ú§ä§ã§ñ §é§ä§Ö§ß§Ú§Ö§Þ SR1, §Ñ §á§à§ä§à§Þ SR2*/
            I2C->STAR1;
            I2C->STAR2;

            /*§±§â§à§é§ä§Ö§Þ §Õ§Ñ§ß§ß§í§Ö, §ß§Ñ§é§Ú§ß§Ñ§ñ §ã §å§Ü§Ñ§Ù§Ñ§ß§ß§à§Ô§à §Ñ§Õ§â§Ö§ã§Ñ*/
            for (uint16_t i = 0; i < Size_data; i++) {
                if (i < Size_data - 1) {
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //§¦§ã§Ý§Ú §Þ§í §ç§à§ä§Ú§Þ §á§â§Ú§ß§ñ§ä§î §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §Ò§Ñ§Û§ä, §ä§à §à§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ ACK
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    *(data + i) = I2C->DATAR; //§¹§ä§Ö§ß§Ú§Ö §Ò§Ñ§Û§ä§Ñ
                } else {
                    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //§¦§ã§Ý§Ú §Þ§í §Ù§ß§Ñ§Ö§Þ, §é§ä§à §ã§Ý§Ö§Õ§å§ð§ë§Ú§Û §á§â§Ú§ß§ñ§ä§í§Û §Ò§Ñ§Û§ä §Ò§å§Õ§Ö§ä §á§à§ã§Ý§Ö§Õ§ß§Ú§Þ, §ä§à §à§ä§á§â§Ñ§Ó§Ú§Þ NACK

                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    //§±§à§Õ§à§Ø§Õ§Ö§Þ, §á§à§Ü§Ñ §ã§Õ§Ó§Ú§Ô§à§Ó§í§Û §â§Ö§Ô§Ú§ã§ä§â §á§à§á§à§Ý§ß§Ú§ä§ã§ñ §ß§à§Ó§í§Þ §Ò§Ñ§Û§ä§à§Þ §Õ§Ñ§ß§ß§í§ç
                    *(data + i) = I2C->DATAR; //§¹§ä§Ö§ß§Ú§Ö §Ò§Ñ§Û§ä§Ñ
                }
            }
            return true;

        } else {
            //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
            CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
            return false;
        }

    } else {
        //§¦§ã§Ý§Ú §å§ã§ä§â§à§Û§ã§ä§Ó§à §ß§Ö §à§ä§à§Ù§Ó§Ñ§Ý§à§ã§î, §á§â§Ú§Ý§Ö§ä§Ú§ä 1 §Ó I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //§°§ã§ä§Ñ§ß§Ñ§Ó§Ý§Ú§Ó§Ñ§Ö§Þ
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //§³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ò§Ú§ä AF
        return false;
    }
}


