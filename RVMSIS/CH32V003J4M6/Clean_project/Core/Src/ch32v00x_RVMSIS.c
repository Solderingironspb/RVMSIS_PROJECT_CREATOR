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
    SET_BIT(RCC->CTLR, RCC_HSION);  // Запустим внутренний RC генератор на 24 МГц
    while (READ_BIT(RCC->CTLR, RCC_HSIRDY) == 0);
    // Дождемся поднятия флага о готовности
    CLEAR_BIT(RCC->CTLR, RCC_HSEBYP);                        // Просто сбросим этот бит в 0(Хотя изначально он и так должен быть в 0).
    CLEAR_BIT(RCC->CTLR, RCC_HSEON);                         // Внешний кварцевый резонатор отсутствует.
    CLEAR_BIT(RCC->CTLR, RCC_CSSON);                         // Выключим CSS
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b00 << RCC_SW_Pos);      // Выберем HSI в качестве System Clock(PLL лучше пока не выбирать, он у нас отключен)
    CLEAR_BIT(RCC->CTLR, RCC_PLLON);                         // Выключим PLL
    MODIFY_REG(RCC->CFGR0, RCC_HPRE, 0b00 << RCC_HPRE_Pos);  // не делим ничего, частота до включения должна быть 24мгц
    // Флеш должен сам делиться на 3
    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_1;
    MODIFY_REG(RCC->CFGR0, RCC_ADCPRE, 0b00100 << RCC_ADCPRE_Pos);  // ADC Prescaler /6, чтоб было 12MHz, т.к. максимальная частота тут 14 MHz
    CLEAR_BIT(RCC->CFGR0, RCC_PLLSRC);                              // HSI не делится и не отправляется в PLL.
    SET_BIT(RCC->CTLR, RCC_PLLON);                                  // Запустим PLL
    // Т.к. PLL уже запущен, выберем его в качестве System Clock:
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b10 << RCC_SW_Pos);  // Выберем PLL в качестве System Clock
    while (READ_BIT(RCC->CTLR, RCC_PLLRDY) == 0);
    // Дожидемся поднятия флага включения PLL
    MODIFY_REG(RCC->CFGR0, RCC_SWS, 0b10 << RCC_SWS_Pos);
}

/*========================= НАСТРОЙКА СИСТЕМНОГО ТАЙМЕРА ==============================*/

/**
 ***************************************************************************************
 *  @breif Настройка SysTick на микросекунды
 *  На этом таймере мы настроим Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */
void RVMSIS_SysTick_Timer_init(void) {
    SysTick->CTLR &= ~(1 << 0);  // Выключим таймер для проведения настроек.
    SysTick->CTLR |= (1 << 1);   // 1: Enable counter interrupts.
    SysTick->CTLR &= ~(1 << 2);  // 0: HCLK for time base.48/8 = 6
    SysTick->CTLR |= (1 << 3);   // 1: Count up to the comparison value and start counting from 0 again
    SysTick->CMP = 5999;         ////Настроим прерывание на частоту в 1 кГц(т.е. сработка будет каждую мс) 18000000 / 18000 = 1000Гц
    SysTick->CNT = 5999;
    NVIC_EnableIRQ(SysTicK_IRQn);
    NVIC_SetPriority(SysTicK_IRQn, 1);
    SysTick->CTLR |= (1 << 0);  // Запустим таймер.
}

/**
 ***************************************************************************************
 *  @breif Настройка Delay и аналог HAL_GetTick()
 ***************************************************************************************
 */
volatile uint32_t SysTimer_ms = 0;         // Переменная, аналогичная HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0;    // Счетчик для функции Delay_ms
volatile uint32_t Timeout_counter_ms = 0;  // Переменная для таймаута функций

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

// Служебная функция
static void RVMSIS_GPIO_MODE_Set(GPIO_TypeDef* GPIO, uint8_t GPIO_Pin, uint8_t Reg, uint8_t Data) {
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

// Служебная функция
static void RVMSIS_GPIO_SPEED_Set(GPIO_TypeDef* GPIO, uint8_t GPIO_Pin, uint8_t Speed) {
    uint8_t Reg = 0;
    if (GPIO_Pin < 8) {
        Reg = 0;
    } else {
        Reg = 1;
    }
    // MODE
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

// Служебная функция
static void RVMSIS_GPIO_CNF_Set(GPIO_TypeDef* GPIO, uint8_t Reg, uint8_t Mode, uint8_t* CNF_Pos) {
    switch (Reg) {
        case (0):
            MODIFY_REG(GPIO->CFGLR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
            break;
        case (1):
            MODIFY_REG(GPIO->CFGHR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
    }
}

// Служебная функция
static void RVMSIS_GPIO_Reg_Set(GPIO_TypeDef* GPIO, uint8_t* GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Reg, uint8_t* CNF_Pos) {
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

void RVMSIS_GPIO_init(GPIO_TypeDef* GPIO, uint8_t GPIO_Pin, uint8_t Configuration_mode, uint8_t Type, uint8_t Speed) {
    uint8_t CNF_Pos = 0;
    if (GPIO == GPIOA) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA);  // Запуск тактирования порта А
    } else if (GPIO == GPIOC) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC);  // Запуск тактирования порта C
    } else if (GPIO == GPIOD) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOD);  // Запуск тактирования порта D
    }

    RVMSIS_GPIO_SPEED_Set(GPIO, GPIO_Pin, Speed);

    if (GPIO_Pin < 8) {
        CNF_Pos = (GPIO_Pin * 4) + 2;
        RVMSIS_GPIO_Reg_Set(GPIO, (uint8_t*)&GPIO_Pin, Configuration_mode, Type, 0, &CNF_Pos);
    } else {
        GPIO_Pin = GPIO_Pin - 8;
        CNF_Pos = (GPIO_Pin * 4) + 2;
        RVMSIS_GPIO_Reg_Set(GPIO, (uint8_t*)&GPIO_Pin, Configuration_mode, Type, 1, &CNF_Pos);
    }
}

/*Таймер 1 для примера*/
void RVMSIS_TIM1_init(void) {
    /*Включим тактирование таймера (страница 48)*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_TIM1);  // Запуск тактирования таймера 1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO);  // Запуск тактирования альтернативных функций

    CLEAR_BIT(TIM1->CTLR1, TIM_UDIS);                                   // Генерировать событие Update
    CLEAR_BIT(TIM1->CTLR1, TIM_URS);                                    // Генерировать прерывание
    CLEAR_BIT(TIM1->CTLR1, TIM_OPM);                                    // One pulse mode off(Счетчик не останавливается при обновлении)
    CLEAR_BIT(TIM1->CTLR1, TIM_DIR);                                    // Считаем вниз
    MODIFY_REG(TIM1->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos);              // Выравнивание по краю
    SET_BIT(TIM1->CTLR1, TIM_ARPE);                                     // Auto-reload preload enable
    MODIFY_REG(TIM1->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos);  // Предделение выключено

    /*Настройка прерываний (Страница 409)*/
    SET_BIT(TIM1->DMAINTENR, TIM_UIE);  // Update interrupt enable

    TIM1->PSC = 200 - 1;
    TIM1->ATRLR = 1000 - 1;

    /*Для работы ШИМ*/
    MODIFY_REG(TIM1->BDTR, TIM_LOCK, 0b00 << TIM_LOCK_Pos);
    SET_BIT(TIM1->BDTR, TIM_AOE);

    NVIC_EnableIRQ(TIM1_UP_IRQn);  // Разрешить прерывания по таймеру 3
    NVIC_SetPriority(TIM1_UP_IRQn, 0);
    SET_BIT(TIM1->CTLR1, TIM_CEN);  // Запуск таймера
}

__WEAK void TIM1_UP_IRQHandler(void) {
    if (READ_BIT(TIM1->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM1->INTFR, TIM_UIF);  // Сбросим флаг прерывания
    }
}

void RVMSIS_TIM1_PWM_CHANNEL1_init(void) {
    /*Настройка ножки PA1 под ШИМ*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA);  // Включим тактирование порта А
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b10 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos);

    /*Настройка шим(Канал 1)*/
    MODIFY_REG(TIM1->CHCTLR1, TIM_CC1S, 0b00 << TIM_CC1S_Pos);   // CC1 channel is configured as output
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC1FE);                         // Fast mode disable
    SET_BIT(TIM1->CHCTLR1, TIM_OC1PE);                           // Preload enable
    MODIFY_REG(TIM1->CHCTLR1, TIM_OC1M, 0b110 << TIM_OC1M_Pos);  // PWM MODE 1
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC1CE);                         // OC1Ref is not affected by the ETRF input

    /*Запуск ШИМ*/
    // 15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM1->CCER, TIM_CC1E);    // On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM1->CCER, TIM_CC1P);  // OC1 active high.

    TIM1->CH1CVR = 512;
}

void RVMSIS_TIM1_PWM_CHANNEL2_init(void) {
    /*Настройка ножки PA1 под ШИМ*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA);  // Включим тактирование порта А
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b10 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos);

    /*Настройка шим(Канал 2)*/
    MODIFY_REG(TIM1->CHCTLR1, TIM_CC2S, 0b00 << TIM_CC1S_Pos);   // CC1 channel is configured as output
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC2FE);                         // Fast mode disable
    SET_BIT(TIM1->CHCTLR1, TIM_OC2PE);                           // Preload enable
    MODIFY_REG(TIM1->CHCTLR1, TIM_OC2M, 0b110 << TIM_OC2M_Pos);  // PWM MODE 1
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC2CE);                         // OC1Ref is not affected by the ETRF input

    /*Запуск ШИМ*/
    // 15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM1->CCER, TIM_CC2E);    // On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM1->CCER, TIM_CC2P);  // OC1 active high.

    TIM1->CH2CVR = 512;
}

void RVMSIS_TIM2_init(void) {
    /*Включим тактирование таймера (страница 48)*/
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_TIM2);  // Запуск тактирования таймера 1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO);  // Запуск тактирования альтернативных функций

    CLEAR_BIT(TIM2->CTLR1, TIM_UDIS);                                   // Генерировать событие Update
    CLEAR_BIT(TIM2->CTLR1, TIM_URS);                                    // Генерировать прерывание
    CLEAR_BIT(TIM2->CTLR1, TIM_OPM);                                    // One pulse mode off(Счетчик не останавливается при обновлении)
    CLEAR_BIT(TIM2->CTLR1, TIM_DIR);                                    // Считаем вниз
    MODIFY_REG(TIM2->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos);              // Выравнивание по краю
    SET_BIT(TIM2->CTLR1, TIM_ARPE);                                     // Auto-reload preload enable
    MODIFY_REG(TIM2->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos);  // Предделение выключено

    /*Настройка прерываний (Страница 409)*/
    SET_BIT(TIM2->DMAINTENR, TIM_UIE);  // Update interrupt enable

    TIM2->PSC = 48 - 1;
    TIM2->ATRLR = 1000 - 1;

    /*Для работы ШИМ*/
    // MODIFY_REG(TIM1->BDTR, TIM_LOCK, 0b00 << TIM_LOCK_Pos);
    // SET_BIT(TIM1->BDTR, TIM_AOE);
    NVIC_EnableIRQ(TIM2_IRQn);  // Разрешить прерывания по таймеру 3
    NVIC_SetPriority(TIM2_IRQn, 2);
    SET_BIT(TIM2->CTLR1, TIM_CEN);  // Запуск таймера
}

__WEAK void TIM2_IRQHandler(void) {
    if (READ_BIT(TIM2->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM2->INTFR, TIM_UIF);  // Сбросим флаг прерывания
    }
}

/*================================= НАСТРОЙКА ADC ============================================*/

/**
 ***************************************************************************************
 *  @breif Analog-to-digital converter (ADC)
 ***************************************************************************************
 */

volatile uint16_t ADC_RAW_Data[2] = {
    0,
};  // Массив, куда будем кидать данные с АЦП

void RVMSIS_ADC_DMA_init(void) {
    // Chapter 11 Direct Memory Access Control (DMA)
    SET_BIT(RCC->AHBPCENR, RCC_AHBPeriph_DMA1);                                     // Включение тактирования DMA1
    DMA1_Channel1->PADDR = (uint32_t)&(ADC1->RDATAR);                               // Задаем адрес периферийного устройства
    DMA1_Channel1->MADDR = (uint32_t)ADC_RAW_Data;                                  // Задаем адрес в памяти, куда будем кидать данные.
    DMA1_Channel1->CNTR = 1;                                                        // Настроим количество данных для передачи. После каждого периферийного события это значение будет уменьшаться.
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PL, 0b00 << DMA_CFGR1_PL_Pos);        // Зададим приоритет канала на высокий
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_DIR);                                  // Чтение будем осуществлять с периферии
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_CIRC);                                   // Настроим DMA в Circular mode
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PSIZE, 0b01 << DMA_CFGR1_PSIZE_Pos);  // Размер данных периферийного устройства 16 бит
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_MSIZE, 0b01 << DMA_CFGR1_MSIZE_Pos);  // Размер данных в памяти 16 бит
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TCIE);                                   // Включим прерывание по полной передаче
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_HTIE);                                 // Отключим прерывание по половинной передаче
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TEIE);                                   // Включим прерывание по ошибке передачи.
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_MINC);                                   // Включим инкрементирование памяти
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_EN);                                     // DMA ON
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_ADC1);   // Включение тактирования ADC1.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA);  // Включение тактирования порта А.

    /*Настройка ножки PA1 и PA2 на аналоговый вход*/
    /*Pin PA1 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b00 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b00 << GPIO_CFGLR_MODE1_Pos);

    /*Pin PA2 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF2, 0b00 << GPIO_CFGLR_CNF2_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE2, 0b00 << GPIO_CFGLR_MODE2_Pos);

    // Прерывание по АЦП: регулярные каналы (вкл/выкл)
    CLEAR_BIT(ADC1->CTLR1, ADC_EOCIE);  // EOC interrupt enabled/disabled. An interrupt is generated when the EOC bit is set

    // Прерывание по АЦП: analog watchdog (вкл/выкл)
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDIE);  // Analog watchdog interrupt disabled

    // Прерывание по АЦП: инжектированные каналы (вкл/выкл)
    CLEAR_BIT(ADC1->CTLR1, ADC_JEOCIE);  // JEOC interrupt disabled

    SET_BIT(ADC1->CTLR1, ADC_SCAN);  // Scan mode enabled

    /* Примечание:
     * Прерывание EOC или JEOC генерируется только в конце преобразования последнего канала,
     * если установлен соответствующий бит EOCIE или JEOCIE.*/

    CLEAR_BIT(ADC1->CTLR1, ADC_AWDSGL);   // Analog watchdog enabled on all channels
    CLEAR_BIT(ADC1->CTLR1, ADC_JAUTO);    // Automatic injected group conversion disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_DISCEN);   // Discontinuous mode on regular channels disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_JDISCEN);  // Discontinuous mode on injected channels disabled
    // MODIFY_REG(ADC1->CTLR1, ADC_DUALMOD, 0b0110 << ADC_DUALMOD_Pos); //0110: Regular simultaneous mode only
    CLEAR_BIT(ADC1->CTLR1, ADC_JAWDEN);  // Analog watchdog disabled on injected channels
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDEN);   // Analog watchdog disabled on regular channels

    // Control register 2 CTLR2
    SET_BIT(ADC1->CTLR2, ADC_ADON);  // Запустить АЦП

    /* Примечание:
     * Если в этот же момент изменяется какой-либо другой бит в этом регистре,
     * кроме ADON, то конверсия не запускается.
     * Это сделано для предотвращения ошибочного преобразования.*/

    SET_BIT(ADC1->CTLR2, ADC_CONT);  // Continuous conversion mode(непрерывные преобразования)
    SET_BIT(ADC1->CTLR2, ADC_CAL);   // Enable calibration
    /*Примечание:
     * Этот бит устанавливается программой для запуска калибровки.
     * Он сбрасывается аппаратно после завершения калибровки.*/

    while (READ_BIT(ADC1->CTLR2, ADC_CAL));
    // Подождем окончания калибровки
    // Delay_ms(10);

    SET_BIT(ADC1->CTLR2, ADC_DMA);                                 // DMA включен
    CLEAR_BIT(ADC1->CTLR2, ADC_ALIGN);                             // Выравнивание по правому краю
    MODIFY_REG(ADC1->CTLR2, ADC_EXTSEL, 0b111 << ADC_EXTSEL_Pos);  // Запускать преобразование программно
    CLEAR_BIT(ADC1->CTLR2, ADC_EXTTRIG);                           // Conversion on external event disabled
    // SET_BIT(ADC1->CTLR2, ADC_SWSTART); //Начать преобразование
    // SET_BIT(ADC1->CTLR2, ADC_TSVREFE);//Temperature sensor and VREFINT channel enabled

    // 12.3.5 ADCx Sample Time Configuration Register 2 (ADCx_SAMPTR2) (x=1/2)
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP1, 0b111 << ADC_SMP1_Pos);  // 239.5 cycles
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP0, 0b111 << ADC_SMP0_Pos);  // 239.5 cycles
    // MODIFY_REG(ADC1->SAMPTR2, ADC_SMP2, 0b111 << ADC_SMP2_Pos);//239.5 cycles
    // MODIFY_REG(ADC1->SAMPTR2, ADC_SMP1, 0b111 << ADC_SMP1_Pos); //239.5 cycles
    // MODIFY_REG(ADC1->SAMPTR1, ADC_SMP17, 0b111 << ADC_SMP17_Pos); //239.5 cycles

    // 12.3.9 ADCx Regular Channel Sequence Register1 (ADCx_RSQR1) (x=1/2)
    MODIFY_REG(ADC1->RSQR1, ADC_L, 0b0000 << ADC_L_Pos);  // 1 преобразование

    // 12.3.11 ADCx Regular Channel Sequence Register 3 (ADCx_RSQR3) (x=1/2)
    MODIFY_REG(ADC1->RSQR3, ADC_SQ1, 0 << ADC_SQ1_Pos);
    // MODIFY_REG(ADC1->RSQR3, ADC_SQ2, 0 << ADC_SQ2_Pos);
    // MODIFY_REG(ADC1->RSQR3, ADC_SQ2, 1 << ADC_SQ2_Pos);
    // NVIC_EnableIRQ(ADC1_2_IRQn); //Разрешить прерывания по АЦП

    // SET_BIT(ADC1->CTLR2, ADC_SWSTART); //Начать преобразование. Не нужно запускать, если Continuous conversion mode(непрерывные преобразования) включен
}

__WEAK void ADC1_2_IRQHandler(void) {
    if (READ_BIT(ADC1->STATR, ADC_EOC)) {
        ADC1->IDATAR1;  // Читаем канал, чтоб сбросить флаг
    }
}
__WEAK void DMA1_Channel1_IRQHandler(void) {
    if (READ_BIT(DMA1->INTFR, DMA_TCIF1)) {
        SET_BIT(DMA1->INTFCR, DMA_CGIF1);  // Сбросим глобальный флаг.
        /*Здесь можно писать код*/

    } else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
        /*Здесь можно сделать какой-то обработчик ошибок*/
        SET_BIT(DMA1->INTFCR, DMA_CGIF1);  // Сбросим глобальный флаг.
    }
}

/*================================= НАСТРОЙКА I2C ============================================*/

/**
 ***************************************************************************************
 *  @breif Inter-integrated circuit (I2C) interface
 ***************************************************************************************
 */

void RVMSIS_I2C_Reset(void) {
    // Сброс настроек I2C
    // 19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    SET_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST);  //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST) == 0);
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST);  //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST));
    /* Примечание: Этот бит можно использовать для повторной инициализации
     * периферийного устройства после ошибки или заблокированного состояния.
     * Например, если бит BUSY установлен и остается заблокированным из-за сбоя на шине,
     * бит SWRST можно использовать для выхода из этого состояния.*/
}

/**
 *************************************************************************************
 *  @breif Функция инициализации шины I2C1. Sm.
 *************************************************************************************
 */

void RVMSIS_I2C1_Init(void) {
    // Настройки тактирования
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC);  // Запуск тактирование порта C
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO);   // Запуск тактирования альтернативных функций
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_I2C1);   // Запуск тактирования I2C1
    // SET_BIT(AFIO->PCFR1,AFIO_PCFR1_I2C1_REMAP); //Включим ремап

    // Настройки ножек SDA и SCL
    // PC1 SDA (I2C Data I/O) Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_CNF1, 0b11 << GPIO_CFGLR_CNF1_Pos);    // Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos);  // Maximum output speed 50 MHz
    // PC2 SCL (I2C clock) Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_CNF2, 0b11 << GPIO_CFGLR_CNF2_Pos);    // Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_MODE2, 0b11 << GPIO_CFGLR_MODE2_Pos);  // Maximum output speed 50 MHz

    // 19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    RVMSIS_I2C_Reset();

    /*Это все для инита не нужно. После сброса итак будет в 0. */
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ALERT);      // Releases SMBA pin high.Alert Response Address Header followed by NACK
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_PEC);        // No PEC transfer
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_POS);        // ACK bit controls the (N)ACK of the current byte being received in the shift register
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ACK);        // No acknowledge returned
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_STOP);       // No Stop generation
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_START);      // No Start generation
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_NOSTRETCH);  // Clock stretching enabled
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ENGC);       // General call disabled. Address 00h is NACKed.
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ENPEC);      // PEC calculation disabled
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_ENARP);      // ARP disable
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SMBTYPE);    // SMBus Device
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SMBUS);      // I2C mode

    // 19.12.2 I2C Control Register 2 (I2Cx_CTLR2) (x=1/2)
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_LAST);                             // Next DMA EOT is not the last transfer
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_DMAEN);                            // DMA requests disabled
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_ITBUFEN);                          // TxE = 1 or RxNE = 1 does not generate any interrupt.
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_ITEVTEN);                          // Event interrupt disabled
    CLEAR_BIT(I2C1->CTLR2, I2C_CTLR2_ITERREN);                          // Error interrupt disabled
    MODIFY_REG(I2C1->CTLR2, I2C_CTLR2_FREQ, 36 << I2C_CTLR2_FREQ_Pos);  // f PCLK1 = 36 Мгц

    // 19.12.3 I2C Address Register 1 (I2Cx_OADDR1) (x=1/2)
    I2C1->OADDR1 = 0;
    // 19.12.4 I2C Address Register2 (I2Cx_OADDR2) (x=1/2)
    I2C1->OADDR2 = 0;

    // 19.12.8 I2C Clock Register (I2Cx_CKCFGR) (x=1/2)
    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS);  // Standard mode I2C
    // SET_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS); //Fast mode I2C

    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_DUTY);  // Fm mode tlow/thigh = 2
    // SET_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 16/9 (see CCR)

    // Расчет CCR. Смотри примеры расчета
    MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 180 << I2C_CKCFGR_CCR_Pos);  // для Sm mode
    // MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 30 << I2C_CKCFGR_CCR_Pos); //для Fm mode. DUTY 0.
    // MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 4 << I2C_CKCFGR_CCR_Pos); //для Fm mode. DUTY 1.

    // 19.12.9 I2C Rise Time Register (I2Cx_RTR) (x=1/2)
    // MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 37 << I2C_RTR_TRISE_Pos);//для Sm mode
    // MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 12 << I2C_RTR_TRISE_Pos); //для Fm mode

    SET_BIT(I2C1->CTLR1, I2C_CTLR1_PE);  // I2C1 enable
}

/**
 *************************************************************************************
 *  @breif Функция сканирования устройства по заданному 7-битному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @retval  Возвращает статус true - если устройство по заданному адресу отозвалось,
 *           false - если устройство по заданному адресу не отвечает
 *************************************************************************************
 */
bool RVMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms) {
    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        // Если шина занята

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            // Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset();  // ресет
            RVMSIS_I2C1_Init();  // повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            // Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            // Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS);  // Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START);  // Отправляем сигнал START

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        // Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }
    }
    // ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью данных в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1);  // Адрес + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        // Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }
    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        // Если устройство отозвалось
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Отправляем сигнал STOP
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;
        return true;
    } else {
        // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Отправляем сигнал STOP
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF
        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif Функция передачи данных по I2C
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @param  *data - Данные, которые будем отправлять
 *  @param  Size_data - Размер, сколько байт будем отправлять.
 *  @retval  Возвращает статус отправки данных. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        // Если шина занята

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            // Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset();  // ресет
            RVMSIS_I2C1_Init();  // повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            // Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            // Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS);  // Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START);  // Стартуем.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        // Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }
    }
    // ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1);  // Адрес + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        // Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }
    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        // Если устройство отозвалось, сбросим бит ADDR
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*Отправим данные*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i);  // Запись байта
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                // Ждем, пока данные загрузятся в регистр сдвига.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем

        return true;

    } else {
        // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif Функция приема данных по I2C
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @param  *data - Куда будем записывать принятые данные
 *  @param  Size_data - Размер, сколько байт будем принимать.
 *  @retval  Возвращает статус приема данных. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        // Если шина занята

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            // Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset();  // ресет
            RVMSIS_I2C1_Init();  // повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            // Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            // Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS);  // Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START);  // Стартуем.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        // Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }
    }
    // ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1 | 1);  // Адрес + команда Read

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        // Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }
    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        // Если устройство отозвалось, сбросим бит ADDR
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*Прочтем данные*/
        for (uint16_t i = 0; i < Size_data; i++) {
            if (i < Size_data - 1) {
                SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK);  // Если мы хотим принять следующий байт, то отправляем ACK

                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    // Ожидаем, пока в сдвиговом регистре появятся данные
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }

                *(data + i) = I2C->DATAR;  // Чтение байта
            } else {
                CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK);  // Если мы знаем, что следующий принятый байт будет последним, то отправим NACK

                SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    // Ожидаем, пока в сдвиговом регистре появятся данные
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }
                *(data + i) = I2C->DATAR;  // Чтение байта
            }
        }
        return true;

    } else {
        // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF
        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif Функция записи в память по указанному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @param  Adress_data - Адрес в памяти, куда будем записывать данные
 *  @param  Size_adress - Размер адреса в байтах. Пример: 1 - 8 битный адрес. 2 - 16 битный адрес.
 *  @param  *data - Данные, которые будем записывать
 *  @param  Size_data - Размер, сколько байт будем записывать.
 *  @retval  Возвращает статус записи. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        // Если шина занята

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            // Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset();  // ресет
            RVMSIS_I2C1_Init();  // повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            // Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            // Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS);  // Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START);  // Стартуем.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        // Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }
    }
    // ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1);  // Адрес + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        // Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }
    }
    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        // Если устройство отозвалось, сбросим бит ADDR
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*Отправим адрес памяти*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*)&Adress_data + (Size_adress - 1 - i));  // Запись байта
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                // Ждем, пока данные загрузятся в регистр сдвига.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF
                    return false;
                }
            }
        }

        /*Будем записывать данные в ячейку памяти, начиная с указанного адреса*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i);  // Запись байта
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                // Ждем, пока данные загрузятся в регистр сдвига.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем

        return true;

    } else {
        // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif Функция чтения из памяти по указанному адресу
 *  @param  *I2C - шина I2C
 *  @param  Adress_Device - Адрес устройства
 *  @param  Adress_data - Адрес в памяти, откуда будем считывать данные
 *  @param  Size_adress - Размер адреса в байтах. Пример: 1 - 8 битный адрес. 2 - 16 битный адрес.
 *  @param  *data - Данные, в которые будем записывать считанную информацию.
 *  @param  Size_data - Размер, сколько байт будем считывать.
 *  @retval  Возвращает статус считывания. True - Успешно. False - Ошибка.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {
    /*-------------------Проверка занятости шины-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        // Если шина занята

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            // Если линия на самом деле свободна, а BUSY висит
            RVMSIS_I2C_Reset();  // ресет
            RVMSIS_I2C1_Init();  // повторная инициализация
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            // Если стоит статус, что мы в мастере
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Отправим сигнал STOP
        }

        if (I2C->CTLR1 != 1) {
            // Если в CR1 что-то лишнее, то перезагрузим I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------Проверка занятости шины-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS);  // Бит ACK управляет (N)ACK текущего байта, принимаемого в сдвиговом регистре.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START);  // Стартуем.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        // Ожидаем до момента, пока не сработает Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }
    }
    // ВНИМАНИЕ!
    /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1);  // Адрес + команда Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        // Ждем, пока адрес отзовется

        if (!Timeout_counter_ms) {
            return false;
        }
    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        // Если устройство отозвалось, сбросим бит ADDR
        /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*Отправим адрес памяти*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*)&Adress_data + (Size_adress - 1 - i));  // Запись байта
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                // Ждем, пока данные загрузятся в регистр сдвига.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF
                    return false;
                }
            }
        }

        // Повторный старт
        SET_BIT(I2C->CTLR1, I2C_CTLR1_START);  // Стартуем.

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
            // Ожидаем до момента, пока не сработает Start condition generated

            if (!Timeout_counter_ms) {
                return false;
            }
        }
        // ВНИМАНИЕ!
        /* Бит I2C_SR1_SB очищается программно путем чтения регистра SR1 с последующей записью в регистр DR или когда PE=0*/
        I2C->STAR1;
        I2C->DATAR = (Adress_Device << 1 | 1);  // Адрес + команда Read

        Timeout_counter_ms = Timeout_ms;
        while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
            // Ждем, пока адрес отзовется

            if (!Timeout_counter_ms) {
                return false;
            }
        }

        if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
            // Если устройство отозвалось, сбросим бит ADDR
            /*Сброс бита ADDR производится чтением SR1, а потом SR2*/
            I2C->STAR1;
            I2C->STAR2;

            /*Прочтем данные, начиная с указанного адреса*/
            for (uint16_t i = 0; i < Size_data; i++) {
                if (i < Size_data - 1) {
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK);  // Если мы хотим принять следующий байт, то отправляем ACK
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    *(data + i) = I2C->DATAR;  // Чтение байта
                } else {
                    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK);  // Если мы знаем, что следующий принятый байт будет последним, то отправим NACK

                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    // Подождем, пока сдвиговый регистр пополнится новым байтом данных
                    *(data + i) = I2C->DATAR;  // Чтение байта
                }
            }
            return true;

        } else {
            // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
            CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF
            return false;
        }

    } else {
        // Если устройство не отозвалось, прилетит 1 в I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);  // Останавливаем
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);  // Сбрасываем бит AF
        return false;
    }
}
