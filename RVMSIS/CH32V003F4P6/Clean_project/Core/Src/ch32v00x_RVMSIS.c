/*
 * ch32v00x_RVMSIS.c
 *
 *  Created on: May 16, 2024
 *      Author: Solderingiron
 */
#include "ch32v00x_RVMSIS.h"
/*============================== НАСТРОЙКА RCC =======================================*/
/**
 ***************************************************************************************
 *  @breif Настройка МК CH32V003J4M6 на частоту 48MHz от внутреннего RC генератора
 *  Внутренний RC генератора на 24 MHz
 *  ADC настроен на 24MHz
 *  SYSCLK 48MHz
 *
 ***************************************************************************************
 */
void RVMSIS_RCC_SystemClock_48MHz(void) {
    SET_BIT(RCC->CTLR, RCC_HSION); //Запустим внутренний RC генератор на 24 МГц
    while (READ_BIT(RCC->CTLR, RCC_HSIRDY) == 0);
    //Дождемся поднятия флага о готовности
    CLEAR_BIT(RCC->CTLR, RCC_HSEBYP);//Просто сбросим этот бит в 0(Хотя изначально он и так должен быть в 0).
    SET_BIT(RCC->CTLR, RCC_HSEON); //Внешний кварцевый резонатор подключен
    while (READ_BIT(RCC->CTLR, RCC_HSERDY) == 0); //Ожидаем поднятия флага готовности внешнего резонатора
    CLEAR_BIT(RCC->CTLR, RCC_CSSON); //Выключим CSS
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b01 << RCC_SW_Pos); //Выберем HSE в качестве System Clock(PLL лучше пока не выбирать, он у нас отключен)
    CLEAR_BIT(RCC->CTLR, RCC_PLLON); //Выключим PLL
    MODIFY_REG(RCC->CFGR0, RCC_HPRE, 0b00 << RCC_HPRE_Pos); //не делим ничего, частота до включения должна быть 24мгц
    //Флеш должен сам делиться на 3
    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t) ((uint32_t) ~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t) FLASH_ACTLR_LATENCY_1;
    MODIFY_REG(RCC->CFGR0, RCC_ADCPRE, 0b00 << RCC_ADCPRE_Pos);    //ADC Prescaler /6, чтоб было 8MHz, т.к. максимальная частота тут 14 MHz
    SET_BIT(RCC->CFGR0, RCC_PLLSRC); //HSE отправляется в PLL
    SET_BIT(RCC->CTLR, RCC_PLLON); //Запустим PLL
    //Т.к. PLL уже запущен, выберем его в качестве System Clock:
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b10 << RCC_SW_Pos);//Выберем PLL в качестве System Clock
    while (READ_BIT(RCC->CTLR, RCC_PLLRDY) == 0);
    //Дожидемся поднятия флага включения PLL
    MODIFY_REG(RCC->CFGR0, RCC_SWS, 0b10 << RCC_SWS_Pos);
}

/*========================= НАСТРОЙКА СИСТЕМНОГО ТАЙМЕРА ==============================*/

uint32_t STick = 0;
/**
 ***************************************************************************************
 *  @breif Настройка SysTick на микросекунды
 *  На этом таймере мы настроим Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */
void RVMSIS_SysTick_Timer_init(void) {
    //SysTick->CTLR &= ~(1 << 0); //Выключим таймер для проведения настроек.
    SysTick->CTLR |= (1 << 0); //Запустим таймер.
    SysTick->CTLR |= (1 << 1); //1: Enable counter interrupts.
    SysTick->CTLR &= ~(1 << 2); //0: HCLK for time base.48/8 = 6
    SysTick->CTLR |= (1 << 3); //1: Re-counting from 0 after counting up to the comparison value, and re-counting from the comparison value after counting down to 0
    SysTick->CTLR |= (1 << 4); //0: Counting up.
    SysTick->CTLR |= (1 << 5); //1: Updated to 0 on up counts, updated to the comparison value on down counts.
    SysTick->CMP = 5999; ////Настроим прерывание на частоту в 1 кГц(т.е. сработка будет каждую мс) 18000000 / 18000 = 1000Гц
    SysTick->CNT = 5999;
    NVIC_EnableIRQ(SysTicK_IRQn);
    NVIC_SetPriority(SysTicK_IRQn, 2);
    STick = SysTick->CTLR;
    while (STick != 0xb) {};
    //SysTick->CTLR |= (1 << 0); //Запустим таймер.

}

/**
 ***************************************************************************************
 *  @breif Настройка Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */
volatile uint32_t SysTimer_ms = 0; //Переменная, аналогичная HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //Счетчик для функции Delay_ms
volatile uint32_t Timeout_counter_ms = 0; //Переменная для таймаута функций

/**
 ******************************************************************************
 *  @breif Прерывание по флагу CNTIF
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
 *  @param   uint32_t Milliseconds - Длина задержки в миллисекундах
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
    Delay_counter_ms = Milliseconds;
    while (Delay_counter_ms != 0);
}

/*============================== НАСТРОЙКА GPIO =======================================*/

//Служебная функция
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

//Служебная функция
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

//Служебная функция
static void RVMSIS_GPIO_CNF_Set(GPIO_TypeDef *GPIO, uint8_t Reg, uint8_t Mode, uint8_t* CNF_Pos) {
    switch (Reg) {
    case (0):
        MODIFY_REG(GPIO->CFGLR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
        break;
    case (1):
        MODIFY_REG(GPIO->CFGHR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
    }
}

//Служебная функция
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
 *  @breif Быстрая конфигурация GPIO
 *  Reference Manual/см. п.9.2 GPIO registers (стр. 171)
 *  Перед настройкой (GPIOs and AFIOs) нужно включить тактирование порта.
 *  @param  *GPIO - Порт GPIO(A, B, C, D, E)
 *  @param  GPIO_Pin - номер пина 0-15
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
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Запуск тактирования порта А
    } else if (GPIO == GPIOC) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //Запуск тактирования порта C
    } else if (GPIO == GPIOD) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOD); //Запуск тактирования порта D
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

/*Таймер 1 для примера*/
void RVMSIS_TIM1_init(void) {
    /*Включим тактирование таймера (страница 48)*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_TIM1); //Запуск тактирования таймера 1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Запуск тактирования альтернативных функций

    CLEAR_BIT(TIM1->CTLR1, TIM_UDIS); //Генерировать событие Update
    CLEAR_BIT(TIM1->CTLR1, TIM_URS); //Генерировать прерывание
    CLEAR_BIT(TIM1->CTLR1, TIM_OPM); //One pulse mode off(Счетчик не останавливается при обновлении)
    CLEAR_BIT(TIM1->CTLR1, TIM_DIR); //Считаем вниз
    MODIFY_REG(TIM1->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos); //Выравнивание по краю
    SET_BIT(TIM1->CTLR1, TIM_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM1->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos); //Предделение выключено

    /*Настройка прерываний (Страница 409)*/
    SET_BIT(TIM1->DMAINTENR, TIM_UIE); //Update interrupt enable

    TIM1->PSC = 1 - 1;
    TIM1->ATRLR = 4095 - 1;

    /*Для работы ШИМ*/
    MODIFY_REG(TIM1->BDTR, TIM_LOCK, 0b00 << TIM_LOCK_Pos);
    SET_BIT(TIM1->BDTR, TIM_AOE);

    //NVIC_EnableIRQ(TIM1_UP_IRQn); //Разрешить прерывания по таймеру 3
    SET_BIT(TIM1->CTLR1, TIM_CEN); //Запуск таймера
}

__WEAK void TIM1_UP_IRQHandler(void) {
    if (READ_BIT(TIM1->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM1->INTFR, TIM_UIF); //Сбросим флаг прерывания
    }
}

void RVMSIS_TIM1_PWM_CHANNEL1_init(void) {
    /*Настройка ножки PA8 под ШИМ*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Включим тактирование порта А
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_CNF8, 0b10 << GPIO_CFGHR_CNF8_Pos);
    MODIFY_REG(GPIOA->CFGHR, GPIO_CFGHR_MODE8, 0b11 << GPIO_CFGHR_MODE8_Pos);

    /*Настройка шим(Канал 1)*/
    MODIFY_REG(TIM1->CHCTLR1, TIM_CC1S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC1FE); //Fast mode disable
    SET_BIT(TIM1->CHCTLR1, TIM_OC1PE); //Preload enable
    MODIFY_REG(TIM1->CHCTLR1, TIM_OC1M, 0b110 << TIM_OC1M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC1CE); //OC1Ref is not affected by the ETRF input

    /*Запуск ШИМ*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM1->CCER, TIM_CC1E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM1->CCER, TIM_CC1P); //OC1 active high.

    TIM1->CH1CVR = 20;
}

void RVMSIS_TIM1_PWM_CHANNEL2_init(void) {
    /*Настройка ножки PA1 под ШИМ*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //Включим тактирование порта А
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b10 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos);

    /*Настройка шим(Канал 2)*/
    MODIFY_REG(TIM1->CHCTLR1, TIM_CC2S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC2FE); //Fast mode disable
    SET_BIT(TIM1->CHCTLR1, TIM_OC2PE); //Preload enable
    MODIFY_REG(TIM1->CHCTLR1, TIM_OC2M, 0b110 << TIM_OC2M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC2CE); //OC1Ref is not affected by the ETRF input

    /*Запуск ШИМ*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM1->CCER, TIM_CC2E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM1->CCER, TIM_CC2P); //OC1 active high.

    TIM1->CH2CVR = 0;
}

void RVMSIS_TIM1_PWM_CHANNEL3_init(void) {
    /*Настройка ножки PC3 под ШИМ*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //Включим тактирование порта C
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_CNF3, 0b10 << GPIO_CFGLR_CNF3_Pos);
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_MODE3, 0b11 << GPIO_CFGLR_MODE3_Pos);

    /*Настройка шим(Канал 3)*/
    MODIFY_REG(TIM1->CHCTLR2, TIM_CC3S, 0b00 << TIM_CC3S_Pos); //CC3 channel is configured as output
    CLEAR_BIT(TIM1->CHCTLR2, TIM_OC3FE); //Fast mode disable
    SET_BIT(TIM1->CHCTLR2, TIM_OC3PE); //Preload enable
    MODIFY_REG(TIM1->CHCTLR2, TIM_OC3M, 0b110 << TIM_OC3M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM1->CHCTLR2, TIM_OC3CE); //OC1Ref is not affected by the ETRF input

    /*Запуск ШИМ*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM1->CCER, TIM_CC3E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM1->CCER, TIM_CC3P); //OC1 active high.

    TIM1->CH3CVR = 0;
}

void RVMSIS_TIM2_PWM_CHANNEL3_init(void) {
    /*Настройка ножки PC0 под ШИМ*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //Включим тактирование порта C
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_CNF0, 0b10 << GPIO_CFGLR_CNF0_Pos);
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_MODE0, 0b11 << GPIO_CFGLR_MODE0_Pos);

    /*Настройка шим(Канал 3)*/
    MODIFY_REG(TIM2->CHCTLR2, TIM_CC3S, 0b00 << TIM_CC3S_Pos); //CC3 channel is configured as output
    CLEAR_BIT(TIM2->CHCTLR2, TIM_OC3FE); //Fast mode disable
    SET_BIT(TIM2->CHCTLR2, TIM_OC3PE); //Preload enable
    MODIFY_REG(TIM2->CHCTLR2, TIM_OC3M, 0b110 << TIM_OC3M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM2->CHCTLR2, TIM_OC3CE); //OC1Ref is not affected by the ETRF input

    /*Запуск ШИМ*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM2->CCER, TIM_CC3E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM2->CCER, TIM_CC3P); //OC1 active high.

    TIM2->CH3CVR = 512;
}

void RVMSIS_TIM2_init(void) {
    /*Включим тактирование таймера (страница 48)*/
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_TIM2); //Запуск тактирования таймера 1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Запуск тактирования альтернативных функций

    CLEAR_BIT(TIM2->CTLR1, TIM_UDIS); //Генерировать событие Update
    CLEAR_BIT(TIM2->CTLR1, TIM_URS); //Генерировать прерывание
    CLEAR_BIT(TIM2->CTLR1, TIM_OPM); //One pulse mode off(Счетчик не останавливается при обновлении)
    CLEAR_BIT(TIM2->CTLR1, TIM_DIR); //Считаем вниз
    MODIFY_REG(TIM2->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos); //Выравнивание по краю
    SET_BIT(TIM2->CTLR1, TIM_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM2->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos); //Предделение выключено

    /*Настройка прерываний (Страница 409)*/
    SET_BIT(TIM2->DMAINTENR, TIM_UIE); //Update interrupt enable

    TIM2->PSC = 1 - 1;
    TIM2->ATRLR = 1000 - 1;

    /*Для работы ШИМ*/
    //MODIFY_REG(TIM1->BDTR, TIM_LOCK, 0b00 << TIM_LOCK_Pos);
    //SET_BIT(TIM1->BDTR, TIM_AOE);

    NVIC_EnableIRQ(TIM2_IRQn); //Разрешить прерывания по таймеру 3
    NVIC_SetPriority(TIM2_IRQn, 2);
    SET_BIT(TIM2->CTLR1, TIM_CEN); //Запуск таймера
}

__WEAK void TIM2_IRQHandler(void) {
    if (READ_BIT(TIM2->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM2->INTFR, TIM_UIF); //Сбросим флаг прерывания
    }
}


/*================================= НАСТРОЙКА USART ============================================*/

/**
 ***************************************************************************************
 *  @breif Universal synchronous asynchronous receiver transmitter (USART)
 ***************************************************************************************
 */

struct USART_name husart1; //Объявляем структуру по USART.(см. ch32v203x_RVMSIS.h)


/**
 ******************************************************************************
 *  @breif Настройка USART1. Параметры 9600 8 N 1
 ******************************************************************************
 */

void RVMSIS_USART1_Init(void) {

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOD); //Включение тактирование порта А
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //Включение альтернативных функций

    //Для конфигурирование ножек UART для Full Duplex нужно использовать Alternate function push-pull(См. п.п. 9.1.11 GPIO configurations for device peripherals стр.111 Reference Manual)
    //Tx - Alternative Function output Push-pull(Maximum output speed 30 MHz) A5
    MODIFY_REG(GPIOD->CFGLR, GPIO_CFGLR_CNF5, 0b10 << GPIO_CFGLR_CNF5_Pos);
    MODIFY_REG(GPIOD->CFGLR, GPIO_CFGLR_MODE5, 0b11 << GPIO_CFGLR_MODE5_Pos);
    //Rx - Input floating A6
    MODIFY_REG(GPIOD->CFGLR, GPIO_CFGLR_CNF6, 0b1 << GPIO_CFGLR_CNF6_Pos);
    MODIFY_REG(GPIOD->CFGLR, GPIO_CFGLR_MODE6, 0b00 << GPIO_CFGLR_MODE6_Pos);

    //Запустим тактирование USART1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_USART1);

    /*Расчет Fractional baud rate generation
     есть формула:
     Tx/Rx baud = fCK/(16*USARTDIV)
     где fCK - Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
     в нашем случае fCK = 48000000
     допустим нам нужна скорость 9600
     9600 = 48000000/(16*USARTDIV)
     Тогда USARTDIV = 48000000/9600*16 = 312.5
     DIV_Mantissa в данном случае будет 312, что есть 0x138
     DIV_Fraction будет, как 0.5*16 = 8, что есть 0x8
     Тогда весь регистр USART->BRR для скорости 9600 будет выглядеть, как 0x1388.
     для примера еще разберем скорость 115200:
     115200 = 72000000/(16*USARTDIV)
     Тогда USARTDIV = 72000000/115200*16 = 39.0625
     DIV_Mantissa в данном случае будет 39, что есть 0x27
     DIV_Fraction будет, как 0.0625*16 = 1, что есть 0x1
     Тогда весь регистр USART->BRR для скорости 115200 будет выглядеть, как 0x271.
     */

    MODIFY_REG(USART1->BRR, USART_BRR_DIV_Mantissa, 0x138 << USART_BRR_DIV_Mantissa_Pos);
    MODIFY_REG(USART1->BRR, USART_BRR_DIV_Fraction, 0x8 << USART_BRR_DIV_Fraction_Pos);

    //18.10.4 USART Control Register1 (USARTx_CTLR1) (x=1/2/3/4/5/6/7/8)
    SET_BIT(USART1->CTLR1, USART_CTLR1_UE);//USART enable
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_M); //Word lenght 1 Start bit, 8 Data bits, n Stop bit
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_WAKE); //Wake up idle Line
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_PCE); //Partity control disabled
    //настройка прерываний
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_PEIE);//partity error interrupt disabled
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_TXEIE); //TXE interrupt is inhibited
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_TCIE); //Transmission complete interrupt disabled
    SET_BIT(USART1->CTLR1, USART_CTLR1_RXNEIE); //Прерывание по приему данных включено
    SET_BIT(USART1->CTLR1, USART_CTLR1_IDLEIE); //Прерывание по флагу IDLE включено
    SET_BIT(USART1->CTLR1, USART_CTLR1_TE); //Transmitter is enabled
    SET_BIT(USART1->CTLR1, USART_CTLR1_RE); //Receiver is enabled and begins searching for a start bit
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_RWU);
    CLEAR_BIT(USART1->CTLR1, USART_CTLR1_SBK);

    //Остальную настройку, не касающуюся стандартного USART, мы пока трогать не будем, но на всякий случай обнулим
    //18.10.5 USART Control Register2 (USARTx_CTLR2) (x=1/2/3/4/5/6/7/8)
    USART1->CTLR2 = 0;
    CLEAR_BIT(USART1->CTLR2, USART_CTLR2_STOP); //1 стоп бит.
    //18.10.6 USART Control Register 3 (USARTx_CTLR3) (x=1/2/3/4/5/6/7/8)
    USART1->CTLR3 = 0;
    //18.10.7 USART Guard Time and Prescaler Register (USARTx_GPR) (x=1/2/3/4/5/6/7/8)
    USART1->GPR = 0;

    NVIC_EnableIRQ(USART1_IRQn); //Включим прерывания по USART1
    NVIC_SetPriority(USART1_IRQn, 0);
}

/**
 ******************************************************************************
 *  @breif Прерывание по USART1
 ******************************************************************************
 */

__WEAK void USART1_IRQHandler(void) {
    if (READ_BIT(USART1->STATR, USART_STATR_RXNE)) {
        //Если пришли данные по USART
        if (husart1.rx_counter < USART_MAX_LEN_RX_BUFFER) { //Если байт прилетело меньше, чем размер буфера
            husart1.rx_buffer[husart1.rx_counter] = USART1->DATAR; //Считаем данные в соответствующую ячейку в rx_buffer
            husart1.rx_counter++; //Увеличим счетчик принятых байт на 1
        } else {
            husart1.rx_counter = 0; //Если больше - сбросим счетчик.
        }
    }
    if (READ_BIT(USART1->STATR, USART_STATR_IDLE)) {
        //Если прилетел флаг IDLE
        USART1->DATAR; //Сбросим флаг IDLE
        husart1.rx_len = husart1.rx_counter; //Узнаем, сколько байт получили
        husart1.rx_counter = 0; //сбросим счетчик приходящих данных
    }
}

/**
 ******************************************************************************
 *  @breif Функция отправки данных по USART
 *  @param  *USART - USART, с которого будут отправляться данные
 *  @param  *data - данные, которые будем отправлять
 *  @param  Size - сколько байт требуется передать
 ******************************************************************************
 */

bool RVMSIS_USART_Transmit(USART_TypeDef* USART, uint8_t* data, uint16_t Size, uint32_t Timeout_ms) {
    for (uint16_t i = 0; i < Size; i++) {
        Timeout_counter_ms = Timeout_ms;
        //Ждем, пока линия не освободится
        while (READ_BIT(USART->STATR, USART_STATR_TXE) == 0) {
            if (!Timeout_counter_ms) {
                return false;
            }
        }
        USART->DATAR = *data++; //Кидаем данные
    }
    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(USART->STATR, USART_STATR_TC) == 0) { //Ожидаем отправки данных
        if (!Timeout_counter_ms) {
            return false;
        }
    };
    return true;
}

