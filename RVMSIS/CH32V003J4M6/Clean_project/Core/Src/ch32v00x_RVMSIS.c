/*
 * ch32v00x_RVMSIS.c
 *
 *  Created on: May 16, 2024
 *      Author: Solderingiron
 */
#include "ch32v00x_RVMSIS.h"
/*============================== ������������������ RCC =======================================*/
/**
 ***************************************************************************************
 *  @breif ���ѧ����ۧܧ� ���� CH32V003J4M6 �ߧ� ��ѧ����� 48MHz ��� �ӧߧ���֧ߧߧ֧ԧ� RC �ԧ֧ߧ֧�ѧ����
 *  ���ߧ���֧ߧߧڧ� RC �ԧ֧ߧ֧�ѧ���� �ߧ� 24 MHz
 *  ADC �ߧѧ����֧� �ߧ� 24MHz
 *  SYSCLK 48MHz
 *
 ***************************************************************************************
 */
void RVMSIS_RCC_SystemClock_48MHz(void) {
    SET_BIT(RCC->CTLR, RCC_HSION); //���ѧ����ڧ� �ӧߧ���֧ߧߧڧ� RC �ԧ֧ߧ֧�ѧ��� �ߧ� 24 ������
    while (READ_BIT(RCC->CTLR, RCC_HSIRDY) == 0);
    //����اէ֧ާ�� ���էߧ��ڧ� ��ݧѧԧ� �� �ԧ���ӧߧ����
    CLEAR_BIT(RCC->CTLR, RCC_HSEBYP);//�������� ��ҧ���ڧ� ����� �ҧڧ� �� 0(������ �ڧ٧ߧѧ�ѧݧ�ߧ� ��� �� ��ѧ� �է�ݧا֧� �ҧ��� �� 0).
    CLEAR_BIT(RCC->CTLR, RCC_HSEON); //���ߧ֧�ߧڧ� �ܧӧѧ��֧ӧ�� ��֧٧�ߧѧ��� ��������ӧ�֧�.
    CLEAR_BIT(RCC->CTLR, RCC_CSSON); //����ܧݧ��ڧ� CSS
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b00 << RCC_SW_Pos); //����ҧ֧�֧� HSI �� �ܧѧ�֧��ӧ� System Clock(PLL �ݧ���� ���ܧ� �ߧ� �ӧ�ҧڧ�ѧ��, ��� �� �ߧѧ� ���ܧݧ��֧�)
    CLEAR_BIT(RCC->CTLR, RCC_PLLON); //����ܧݧ��ڧ� PLL
    MODIFY_REG(RCC->CFGR0, RCC_HPRE, 0b00 << RCC_HPRE_Pos); //�ߧ� �է֧ݧڧ� �ߧڧ�֧ԧ�, ��ѧ����� �է� �ӧܧݧ��֧ߧڧ� �է�ݧاߧ� �ҧ��� 24�ާԧ�
    //���ݧ֧� �է�ݧا֧� ��ѧ� �է֧ݧڧ���� �ߧ� 3
    /* Flash 0 wait state */
    FLASH->ACTLR &= (uint32_t) ((uint32_t) ~FLASH_ACTLR_LATENCY);
    FLASH->ACTLR |= (uint32_t) FLASH_ACTLR_LATENCY_1;
    MODIFY_REG(RCC->CFGR0, RCC_ADCPRE, 0b00100 << RCC_ADCPRE_Pos);    //ADC Prescaler /6, ����� �ҧ�ݧ� 12MHz, ��.��. �ާѧܧ�ڧާѧݧ�ߧѧ� ��ѧ����� ���� 14 MHz
    CLEAR_BIT(RCC->CFGR0, RCC_PLLSRC); //HSI �ߧ� �է֧ݧڧ��� �� �ߧ� �����ѧӧݧ�֧��� �� PLL.
    SET_BIT(RCC->CTLR, RCC_PLLON); //���ѧ����ڧ� PLL
    //��.��. PLL ��ا� �٧ѧ���֧�, �ӧ�ҧ֧�֧� �֧ԧ� �� �ܧѧ�֧��ӧ� System Clock:
    MODIFY_REG(RCC->CFGR0, RCC_SW, 0b10 << RCC_SW_Pos);//����ҧ֧�֧� PLL �� �ܧѧ�֧��ӧ� System Clock
    while (READ_BIT(RCC->CTLR, RCC_PLLRDY) == 0);
    //����اڧէ֧ާ�� ���էߧ��ڧ� ��ݧѧԧ� �ӧܧݧ��֧ߧڧ� PLL
    MODIFY_REG(RCC->CFGR0, RCC_SWS, 0b10 << RCC_SWS_Pos);
}

/*========================= ������������������ �������������������� �������������� ==============================*/

/**
 ***************************************************************************************
 *  @breif ���ѧ����ۧܧ� SysTick �ߧ� �ާڧܧ���֧ܧ�ߧէ�
 *  ���� ����� ��ѧۧާ֧�� �ާ� �ߧѧ����ڧ� Delay �� �ѧߧѧݧ�� HAL_GetTick()
 ***************************************************************************************
 */
void RVMSIS_SysTick_Timer_init(void) {
    SysTick->CTLR &= ~(1 << 0); //����ܧݧ��ڧ� ��ѧۧާ֧� �էݧ� ����ӧ֧է֧ߧڧ� �ߧѧ����֧�.
    SysTick->CTLR |= (1 << 1); //1: Enable counter interrupts.
    SysTick->CTLR &= ~(1 << 2); //0: HCLK for time base.48/8 = 6
    SysTick->CTLR |= (1 << 3); //1: Count up to the comparison value and start counting from 0 again
    SysTick->CMP = 5999; ////���ѧ����ڧ� ���֧��ӧѧߧڧ� �ߧ� ��ѧ����� �� 1 �ܧ���(��.��. ���ѧҧ��ܧ� �ҧ�է֧� �ܧѧاէ�� �ާ�) 18000000 / 18000 = 1000����
    SysTick->CNT = 5999;
    NVIC_EnableIRQ(SysTicK_IRQn);
    NVIC_SetPriority(SysTicK_IRQn, 1);
    SysTick->CTLR |= (1 << 0); //���ѧ����ڧ� ��ѧۧާ֧�.
}

/**
 ***************************************************************************************
 *  @breif ���ѧ����ۧܧ� Delay �� �ѧߧѧݧ�� HAL_GetTick()
 ***************************************************************************************
 */
volatile uint32_t SysTimer_ms = 0; //���֧�֧ާ֧ߧߧѧ�, �ѧߧѧݧ�ԧڧ�ߧѧ� HAL_GetTick()
volatile uint32_t Delay_counter_ms = 0; //����֧��ڧ� �էݧ� ���ߧܧ�ڧ� Delay_ms
volatile uint32_t Timeout_counter_ms = 0; //���֧�֧ާ֧ߧߧѧ� �էݧ� ��ѧۧާѧ��� ���ߧܧ�ڧ�

/**
 ******************************************************************************
 *  @breif ����֧��ӧѧߧڧ� ��� ��ݧѧԧ� CNTIF
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
 *  @param   uint32_t Milliseconds - ���ݧڧߧ� �٧ѧէ֧�اܧ� �� �ާڧݧݧڧ�֧ܧ�ߧէѧ�
 ******************************************************************************
 */
void Delay_ms(uint32_t Milliseconds) {
    Delay_counter_ms = Milliseconds;
    while (Delay_counter_ms != 0);
}

/*============================== ������������������ GPIO =======================================*/

//���ݧ�ا֧ҧߧѧ� ���ߧܧ�ڧ�
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

//���ݧ�ا֧ҧߧѧ� ���ߧܧ�ڧ�
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

//���ݧ�ا֧ҧߧѧ� ���ߧܧ�ڧ�
static void RVMSIS_GPIO_CNF_Set(GPIO_TypeDef *GPIO, uint8_t Reg, uint8_t Mode, uint8_t* CNF_Pos) {
    switch (Reg) {
    case (0):
        MODIFY_REG(GPIO->CFGLR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
        break;
    case (1):
        MODIFY_REG(GPIO->CFGHR, (0x3UL << *CNF_Pos), Mode << *CNF_Pos);
    }
}

//���ݧ�ا֧ҧߧѧ� ���ߧܧ�ڧ�
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
 *  @breif �������ѧ� �ܧ�ߧ�ڧԧ��ѧ�ڧ� GPIO
 *  Reference Manual/���. ��.9.2 GPIO registers (����. 171)
 *  ���֧�֧� �ߧѧ����ۧܧ�� (GPIOs and AFIOs) �ߧ�اߧ� �ӧܧݧ��ڧ�� ��ѧܧ�ڧ��ӧѧߧڧ� ������.
 *  @param  *GPIO - ������ GPIO(A, B, C, D, E)
 *  @param  GPIO_Pin - �ߧ�ާ֧� ��ڧߧ� 0-15
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
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��
    } else if (GPIO == GPIOC) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� ������ C
    } else if (GPIO == GPIOD) {
        SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOD); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� ������ D
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

/*���ѧۧާ֧� 1 �էݧ� ���ڧާ֧��*/
void RVMSIS_TIM1_init(void) {
    /*���ܧݧ��ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ��ѧۧާ֧�� (����ѧߧڧ�� 48)*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_TIM1); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� ��ѧۧާ֧�� 1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�

    CLEAR_BIT(TIM1->CTLR1, TIM_UDIS); //���֧ߧ֧�ڧ��ӧѧ�� ���ҧ��ڧ� Update
    CLEAR_BIT(TIM1->CTLR1, TIM_URS); //���֧ߧ֧�ڧ��ӧѧ�� ���֧��ӧѧߧڧ�
    CLEAR_BIT(TIM1->CTLR1, TIM_OPM); //One pulse mode off(����֧��ڧ� �ߧ� ����ѧߧѧӧݧڧӧѧ֧��� ���� ��ҧߧ�ӧݧ֧ߧڧ�)
    CLEAR_BIT(TIM1->CTLR1, TIM_DIR); //����ڧ�ѧ֧� �ӧߧڧ�
    MODIFY_REG(TIM1->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos); //�����ѧӧߧڧӧѧߧڧ� ��� �ܧ�ѧ�
    SET_BIT(TIM1->CTLR1, TIM_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM1->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos); //����֧էէ֧ݧ֧ߧڧ� �ӧ�ܧݧ��֧ߧ�

    /*���ѧ����ۧܧ� ���֧��ӧѧߧڧ� (�����ѧߧڧ�� 409)*/
    SET_BIT(TIM1->DMAINTENR, TIM_UIE); //Update interrupt enable

    TIM1->PSC = 200 - 1;
    TIM1->ATRLR = 1000 - 1;

    /*���ݧ� ��ѧҧ��� ������*/
    MODIFY_REG(TIM1->BDTR, TIM_LOCK, 0b00 << TIM_LOCK_Pos);
    SET_BIT(TIM1->BDTR, TIM_AOE);

    NVIC_EnableIRQ(TIM1_UP_IRQn); //���ѧ٧�֧�ڧ�� ���֧��ӧѧߧڧ� ��� ��ѧۧާ֧�� 3
    NVIC_SetPriority(TIM1_UP_IRQn, 0);
    SET_BIT(TIM1->CTLR1, TIM_CEN); //���ѧ���� ��ѧۧާ֧��
}

__WEAK void TIM1_UP_IRQHandler(void) {
    if (READ_BIT(TIM1->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM1->INTFR, TIM_UIF); //���ҧ���ڧ� ��ݧѧ� ���֧��ӧѧߧڧ�
    }
}

void RVMSIS_TIM1_PWM_CHANNEL1_init(void) {
    /*���ѧ����ۧܧ� �ߧ�اܧ� PA1 ���� ������*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ܧݧ��ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b10 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos);

    /*���ѧ����ۧܧ� ��ڧ�(���ѧߧѧ� 1)*/
    MODIFY_REG(TIM1->CHCTLR1, TIM_CC1S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC1FE); //Fast mode disable
    SET_BIT(TIM1->CHCTLR1, TIM_OC1PE); //Preload enable
    MODIFY_REG(TIM1->CHCTLR1, TIM_OC1M, 0b110 << TIM_OC1M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC1CE); //OC1Ref is not affected by the ETRF input

    /*���ѧ���� ������*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM1->CCER, TIM_CC1E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM1->CCER, TIM_CC1P); //OC1 active high.

    TIM1->CH1CVR = 512;
}

void RVMSIS_TIM1_PWM_CHANNEL2_init(void) {
    /*���ѧ����ۧܧ� �ߧ�اܧ� PA1 ���� ������*/
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ܧݧ��ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b10 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos);

    /*���ѧ����ۧܧ� ��ڧ�(���ѧߧѧ� 2)*/
    MODIFY_REG(TIM1->CHCTLR1, TIM_CC2S, 0b00 << TIM_CC1S_Pos); //CC1 channel is configured as output
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC2FE); //Fast mode disable
    SET_BIT(TIM1->CHCTLR1, TIM_OC2PE); //Preload enable
    MODIFY_REG(TIM1->CHCTLR1, TIM_OC2M, 0b110 << TIM_OC2M_Pos); //PWM MODE 1
    CLEAR_BIT(TIM1->CHCTLR1, TIM_OC2CE); //OC1Ref is not affected by the ETRF input

    /*���ѧ���� ������*/
    //15.4.9 TIMx capture/compare enable register (TIMx_CCER)
    SET_BIT(TIM1->CCER, TIM_CC2E);//On - OC1 signal is output on the corresponding output pin.
    CLEAR_BIT(TIM1->CCER, TIM_CC2P); //OC1 active high.

    TIM1->CH2CVR = 512;
}

void RVMSIS_TIM2_init(void) {
    /*���ܧݧ��ڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ��ѧۧާ֧�� (����ѧߧڧ�� 48)*/
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_TIM2); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� ��ѧۧާ֧�� 1
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�

    CLEAR_BIT(TIM2->CTLR1, TIM_UDIS); //���֧ߧ֧�ڧ��ӧѧ�� ���ҧ��ڧ� Update
    CLEAR_BIT(TIM2->CTLR1, TIM_URS); //���֧ߧ֧�ڧ��ӧѧ�� ���֧��ӧѧߧڧ�
    CLEAR_BIT(TIM2->CTLR1, TIM_OPM); //One pulse mode off(����֧��ڧ� �ߧ� ����ѧߧѧӧݧڧӧѧ֧��� ���� ��ҧߧ�ӧݧ֧ߧڧ�)
    CLEAR_BIT(TIM2->CTLR1, TIM_DIR); //����ڧ�ѧ֧� �ӧߧڧ�
    MODIFY_REG(TIM2->CTLR1, TIM_CMS, 0b00 << TIM_CMS_Pos); //�����ѧӧߧڧӧѧߧڧ� ��� �ܧ�ѧ�
    SET_BIT(TIM2->CTLR1, TIM_ARPE); //Auto-reload preload enable
    MODIFY_REG(TIM2->CTLR1, TIM_CTLR1_CKD, 0b00 << TIM_CTLR1_CKD_Pos); //����֧էէ֧ݧ֧ߧڧ� �ӧ�ܧݧ��֧ߧ�

    /*���ѧ����ۧܧ� ���֧��ӧѧߧڧ� (�����ѧߧڧ�� 409)*/
    SET_BIT(TIM2->DMAINTENR, TIM_UIE); //Update interrupt enable

    TIM2->PSC = 48 - 1;
    TIM2->ATRLR = 1000 - 1;

    /*���ݧ� ��ѧҧ��� ������*/
    //MODIFY_REG(TIM1->BDTR, TIM_LOCK, 0b00 << TIM_LOCK_Pos);
    //SET_BIT(TIM1->BDTR, TIM_AOE);
    NVIC_EnableIRQ(TIM2_IRQn); //���ѧ٧�֧�ڧ�� ���֧��ӧѧߧڧ� ��� ��ѧۧާ֧�� 3
    NVIC_SetPriority(TIM2_IRQn, 2);
    SET_BIT(TIM2->CTLR1, TIM_CEN); //���ѧ���� ��ѧۧާ֧��
}

__WEAK void TIM2_IRQHandler(void) {
    if (READ_BIT(TIM2->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM2->INTFR, TIM_UIF); //���ҧ���ڧ� ��ݧѧ� ���֧��ӧѧߧڧ�
    }
}

/*================================= ������������������ ADC ============================================*/

/**
 ***************************************************************************************
 *  @breif Analog-to-digital converter (ADC)
 ***************************************************************************************
 */

volatile uint16_t ADC_RAW_Data[2] = { 0, }; //���ѧ��ڧ�, �ܧ�է� �ҧ�է֧� �ܧڧէѧ�� �էѧߧߧ�� �� ������

void RVMSIS_ADC_DMA_init(void) {
    //Chapter 11 Direct Memory Access Control (DMA)
    SET_BIT(RCC->AHBPCENR, RCC_AHBPeriph_DMA1); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� DMA1
    DMA1_Channel1->PADDR = (uint32_t) &(ADC1->RDATAR); //���ѧէѧ֧� �ѧէ�֧� ��֧�ڧ�֧�ڧۧߧ�ԧ� ������ۧ��ӧ�
    DMA1_Channel1->MADDR = (uint32_t) ADC_RAW_Data; //���ѧէѧ֧� �ѧէ�֧� �� ��ѧާ���, �ܧ�է� �ҧ�է֧� �ܧڧէѧ�� �էѧߧߧ��.
    DMA1_Channel1->CNTR = 1; //���ѧ����ڧ� �ܧ�ݧڧ�֧��ӧ� �էѧߧߧ�� �էݧ� ��֧�֧էѧ��. �����ݧ� �ܧѧاէ�ԧ� ��֧�ڧ�֧�ڧۧߧ�ԧ� ���ҧ��ڧ� ���� �٧ߧѧ�֧ߧڧ� �ҧ�է֧� ��ާ֧ߧ��ѧ����.
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PL, 0b00 << DMA_CFGR1_PL_Pos); //���ѧէѧէڧ� ���ڧ��ڧ�֧� �ܧѧߧѧݧ� �ߧ� �ӧ���ܧڧ�
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_DIR); //����֧ߧڧ� �ҧ�է֧� �����֧��ӧݧ��� �� ��֧�ڧ�֧�ڧ�
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_CIRC); //���ѧ����ڧ� DMA �� Circular mode
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_PSIZE, 0b01 << DMA_CFGR1_PSIZE_Pos); //���ѧ٧ާ֧� �էѧߧߧ�� ��֧�ڧ�֧�ڧۧߧ�ԧ� ������ۧ��ӧ� 16 �ҧڧ�
    MODIFY_REG(DMA1_Channel1->CFGR, DMA_CFGR1_MSIZE, 0b01 << DMA_CFGR1_MSIZE_Pos); //���ѧ٧ާ֧� �էѧߧߧ�� �� ��ѧާ��� 16 �ҧڧ�
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TCIE); //���ܧݧ��ڧ� ���֧��ӧѧߧڧ� ��� ���ݧߧ�� ��֧�֧էѧ��
    CLEAR_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_HTIE); //����ܧݧ��ڧ� ���֧��ӧѧߧڧ� ��� ���ݧ�ӧڧߧߧ�� ��֧�֧էѧ��
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_TEIE); //���ܧݧ��ڧ� ���֧��ӧѧߧڧ� ��� ���ڧҧܧ� ��֧�֧էѧ��.
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_MINC); //���ܧݧ��ڧ� �ڧߧܧ�֧ާ֧ߧ�ڧ��ӧѧߧڧ� ��ѧާ���
    SET_BIT(DMA1_Channel1->CFGR, DMA_CFGR1_EN); //DMA ON
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_ADC1); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ADC1.
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOA); //���ܧݧ��֧ߧڧ� ��ѧܧ�ڧ��ӧѧߧڧ� ������ ��.

    /*���ѧ����ۧܧ� �ߧ�اܧ� PA1 �� PA2 �ߧ� �ѧߧѧݧ�ԧ�ӧ�� �ӧ���*/
    /*Pin PA1 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF1, 0b00 << GPIO_CFGLR_CNF1_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE1, 0b00 << GPIO_CFGLR_MODE1_Pos);

    /*Pin PA2 - Analog*/
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_CNF2, 0b00 << GPIO_CFGLR_CNF2_Pos);
    MODIFY_REG(GPIOA->CFGLR, GPIO_CFGLR_MODE2, 0b00 << GPIO_CFGLR_MODE2_Pos);

    //����֧��ӧѧߧڧ� ��� ������: ��֧ԧ�ݧ��ߧ�� �ܧѧߧѧݧ� (�ӧܧ�/�ӧ�ܧ�)
    CLEAR_BIT(ADC1->CTLR1, ADC_EOCIE);//EOC interrupt enabled/disabled. An interrupt is generated when the EOC bit is set

    //����֧��ӧѧߧڧ� ��� ������: analog watchdog (�ӧܧ�/�ӧ�ܧ�)
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDIE);//Analog watchdog interrupt disabled

    //����֧��ӧѧߧڧ� ��� ������: �ڧߧا֧ܧ�ڧ��ӧѧߧߧ�� �ܧѧߧѧݧ� (�ӧܧ�/�ӧ�ܧ�)
    CLEAR_BIT(ADC1->CTLR1, ADC_JEOCIE);//JEOC interrupt disabled

    SET_BIT(ADC1->CTLR1, ADC_SCAN); //Scan mode enabled

    /* ����ڧާ֧�ѧߧڧ�:
     * ����֧��ӧѧߧڧ� EOC �ڧݧ� JEOC �ԧ֧ߧ֧�ڧ��֧��� ���ݧ�ܧ� �� �ܧ�ߧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ� ����ݧ֧էߧ֧ԧ� �ܧѧߧѧݧ�,
     * �֧�ݧ� ����ѧߧ�ӧݧ֧� �����ӧ֧���ӧ���ڧ� �ҧڧ� EOCIE �ڧݧ� JEOCIE.*/

    CLEAR_BIT(ADC1->CTLR1, ADC_AWDSGL); //Analog watchdog enabled on all channels
    CLEAR_BIT(ADC1->CTLR1, ADC_JAUTO); //Automatic injected group conversion disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_DISCEN); //Discontinuous mode on regular channels disabled
    CLEAR_BIT(ADC1->CTLR1, ADC_JDISCEN); //Discontinuous mode on injected channels disabled
    //MODIFY_REG(ADC1->CTLR1, ADC_DUALMOD, 0b0110 << ADC_DUALMOD_Pos); //0110: Regular simultaneous mode only
    CLEAR_BIT(ADC1->CTLR1, ADC_JAWDEN);//Analog watchdog disabled on injected channels
    CLEAR_BIT(ADC1->CTLR1, ADC_AWDEN); //Analog watchdog disabled on regular channels

    //Control register 2 CTLR2
    SET_BIT(ADC1->CTLR2, ADC_ADON);//���ѧ����ڧ�� ������

    /* ����ڧާ֧�ѧߧڧ�:
     * ����ݧ� �� ����� �ا� �ާ�ާ֧ߧ� �ڧ٧ާ֧ߧ�֧��� �ܧѧܧ��-�ݧڧҧ� �է��ԧ�� �ҧڧ� �� ����� ��֧ԧڧ����,
     * �ܧ��ާ� ADON, ��� �ܧ�ߧӧ֧��ڧ� �ߧ� �٧ѧ���ܧѧ֧���.
     * ����� ��է֧ݧѧߧ� �էݧ� ���֧է��ӧ�ѧ�֧ߧڧ� ���ڧҧ��ߧ�ԧ� ���֧�ҧ�ѧ٧�ӧѧߧڧ�.*/

    SET_BIT(ADC1->CTLR2, ADC_CONT); //Continuous conversion mode(�ߧ֧��֧��ӧߧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ�)
    SET_BIT(ADC1->CTLR2, ADC_CAL); //Enable calibration
    /*����ڧާ֧�ѧߧڧ�:
     * ������ �ҧڧ� ����ѧߧѧӧݧڧӧѧ֧��� ����ԧ�ѧާާ�� �էݧ� �٧ѧ���ܧ� �ܧѧݧڧҧ��ӧܧ�.
     * ���� ��ҧ�ѧ��ӧѧ֧��� �ѧ��ѧ�ѧ�ߧ� ����ݧ� �٧ѧӧ֧��֧ߧڧ� �ܧѧݧڧҧ��ӧܧ�.*/

    while (READ_BIT(ADC1->CTLR2, ADC_CAL));
    //����է�اէ֧� ��ܧ�ߧ�ѧߧڧ� �ܧѧݧڧҧ��ӧܧ�
    //Delay_ms(10);

    SET_BIT(ADC1->CTLR2, ADC_DMA);//DMA �ӧܧݧ��֧�
    CLEAR_BIT(ADC1->CTLR2, ADC_ALIGN); //�����ѧӧߧڧӧѧߧڧ� ��� ���ѧӧ�ާ� �ܧ�ѧ�
    MODIFY_REG(ADC1->CTLR2, ADC_EXTSEL, 0b111 << ADC_EXTSEL_Pos); //���ѧ���ܧѧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ� ����ԧ�ѧާާߧ�
    CLEAR_BIT(ADC1->CTLR2, ADC_EXTTRIG); //Conversion on external event disabled
    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //���ѧ�ѧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ�
    //SET_BIT(ADC1->CTLR2, ADC_TSVREFE);//Temperature sensor and VREFINT channel enabled

    // 12.3.5 ADCx Sample Time Configuration Register 2 (ADCx_SAMPTR2) (x=1/2)
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP1, 0b111 << ADC_SMP1_Pos);//239.5 cycles
    MODIFY_REG(ADC1->SAMPTR2, ADC_SMP0, 0b111 << ADC_SMP0_Pos);//239.5 cycles
    //MODIFY_REG(ADC1->SAMPTR2, ADC_SMP2, 0b111 << ADC_SMP2_Pos);//239.5 cycles
    //MODIFY_REG(ADC1->SAMPTR2, ADC_SMP1, 0b111 << ADC_SMP1_Pos); //239.5 cycles
    //MODIFY_REG(ADC1->SAMPTR1, ADC_SMP17, 0b111 << ADC_SMP17_Pos); //239.5 cycles

    //12.3.9 ADCx Regular Channel Sequence Register1 (ADCx_RSQR1) (x=1/2)
    MODIFY_REG(ADC1->RSQR1, ADC_L, 0b0000 << ADC_L_Pos);//1 ���֧�ҧ�ѧ٧�ӧѧߧڧ�

    //12.3.11 ADCx Regular Channel Sequence Register 3 (ADCx_RSQR3) (x=1/2)
    MODIFY_REG(ADC1->RSQR3, ADC_SQ1, 0 << ADC_SQ1_Pos);
    //MODIFY_REG(ADC1->RSQR3, ADC_SQ2, 0 << ADC_SQ2_Pos);
    //MODIFY_REG(ADC1->RSQR3, ADC_SQ2, 1 << ADC_SQ2_Pos);
    //NVIC_EnableIRQ(ADC1_2_IRQn); //���ѧ٧�֧�ڧ�� ���֧��ӧѧߧڧ� ��� ������

    //SET_BIT(ADC1->CTLR2, ADC_SWSTART); //���ѧ�ѧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ�. ���� �ߧ�اߧ� �٧ѧ���ܧѧ��, �֧�ݧ� Continuous conversion mode(�ߧ֧��֧��ӧߧ�� ���֧�ҧ�ѧ٧�ӧѧߧڧ�) �ӧܧݧ��֧�

}

__WEAK void ADC1_2_IRQHandler(void) {
    if (READ_BIT(ADC1->STATR, ADC_EOC)) {
        ADC1->IDATAR1; //���ڧ�ѧ֧� �ܧѧߧѧ�, ����� ��ҧ���ڧ�� ��ݧѧ�
    }

}
__WEAK void DMA1_Channel1_IRQHandler(void) {
    if (READ_BIT(DMA1->INTFR, DMA_TCIF1)) {
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //���ҧ���ڧ� �ԧݧ�ҧѧݧ�ߧ�� ��ݧѧ�.
        /*���է֧�� �ާ�اߧ� ��ڧ�ѧ�� �ܧ��*/

    } else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
        /*���է֧�� �ާ�اߧ� ��է֧ݧѧ�� �ܧѧܧ��-��� ��ҧ�ѧҧ���ڧ� ���ڧҧ��*/
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //���ҧ���ڧ� �ԧݧ�ҧѧݧ�ߧ�� ��ݧѧ�.
    }
}



/*================================= ������������������ I2C ============================================*/

/**
 ***************************************************************************************
 *  @breif Inter-integrated circuit (I2C) interface
 ***************************************************************************************
 */

void RVMSIS_I2C_Reset(void) {
    //���ҧ��� �ߧѧ����֧� I2C
    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    SET_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST) == 0);
    CLEAR_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST); //: I2C Peripheral not under reset
    while (READ_BIT(I2C1->CTLR1, I2C_CTLR1_SWRST));
    /* ����ڧާ֧�ѧߧڧ�: ������ �ҧڧ� �ާ�اߧ� �ڧ���ݧ�٧�ӧѧ�� �էݧ� ���ӧ���ߧ�� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
     * ��֧�ڧ�֧�ڧۧߧ�ԧ� ������ۧ��ӧ� ����ݧ� ���ڧҧܧ� �ڧݧ� �٧ѧҧݧ�ܧڧ��ӧѧߧߧ�ԧ� �������ߧڧ�.
     * ���ѧ��ڧާ֧�, �֧�ݧ� �ҧڧ� BUSY ����ѧߧ�ӧݧ֧� �� ����ѧ֧��� �٧ѧҧݧ�ܧڧ��ӧѧߧߧ�� �ڧ�-�٧� ��ҧ�� �ߧ� ��ڧߧ�,
     * �ҧڧ� SWRST �ާ�اߧ� �ڧ���ݧ�٧�ӧѧ�� �էݧ� �ӧ���է� �ڧ� ����ԧ� �������ߧڧ�.*/
}

/**
 *************************************************************************************
 *  @breif ����ߧܧ�ڧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ� ��ڧߧ� I2C1. Sm.
 *************************************************************************************
 */

void RVMSIS_I2C1_Init(void) {
    //���ѧ����ۧܧ� ��ѧܧ�ڧ��ӧѧߧڧ�
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOC); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� ������ C
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_AFIO); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� �ѧݧ��֧�ߧѧ�ڧӧߧ�� ���ߧܧ�ڧ�
    SET_BIT(RCC->APB1PCENR, RCC_APB1Periph_I2C1); //���ѧ���� ��ѧܧ�ڧ��ӧѧߧڧ� I2C1
    //SET_BIT(AFIO->PCFR1,AFIO_PCFR1_I2C1_REMAP); //���ܧݧ��ڧ� ��֧ާѧ�

    //���ѧ����ۧܧ� �ߧ�ا֧� SDA �� SCL
    //PC1 SDA (I2C Data I/O) Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_CNF1, 0b11 << GPIO_CFGLR_CNF1_Pos);//Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_MODE1, 0b11 << GPIO_CFGLR_MODE1_Pos); //Maximum output speed 50 MHz
    //PC2 SCL (I2C clock) Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_CNF2, 0b11 << GPIO_CFGLR_CNF2_Pos);//Alternate function open drain
    MODIFY_REG(GPIOC->CFGLR, GPIO_CFGLR_MODE2, 0b11 << GPIO_CFGLR_MODE2_Pos); //Maximum output speed 50 MHz

    //19.12.1 I2C Control Register (I2Cx_CTLR1) (x=1/2)
    RVMSIS_I2C_Reset();

    /*����� �ӧ�� �էݧ� �ڧߧڧ�� �ߧ� �ߧ�اߧ�. �����ݧ� ��ҧ���� �ڧ�ѧ� �ҧ�է֧� �� 0. */
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
    MODIFY_REG(I2C1->CTLR2, I2C_CTLR2_FREQ, 36 << I2C_CTLR2_FREQ_Pos); //f PCLK1 = 36 ���ԧ�

    //19.12.3 I2C Address Register 1 (I2Cx_OADDR1) (x=1/2)
    I2C1->OADDR1 = 0;
    //19.12.4 I2C Address Register2 (I2Cx_OADDR2) (x=1/2)
    I2C1->OADDR2 = 0;

    //19.12.8 I2C Clock Register (I2Cx_CKCFGR) (x=1/2)
    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS);//Standard mode I2C
    //SET_BIT(I2C1->CKCFGR, I2C_CKCFGR_FS); //Fast mode I2C

    CLEAR_BIT(I2C1->CKCFGR, I2C_CKCFGR_DUTY);//Fm mode tlow/thigh = 2
    //SET_BIT(I2C1->CCR, I2C_CCR_DUTY); //Fm mode tlow/thigh = 16/9 (see CCR)

    //���ѧ��֧� CCR. ���ާ���� ���ڧާ֧�� ��ѧ��֧��
    MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 180 << I2C_CKCFGR_CCR_Pos);//�էݧ� Sm mode
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 30 << I2C_CKCFGR_CCR_Pos); //�էݧ� Fm mode. DUTY 0.
    //MODIFY_REG(I2C1->CKCFGR, I2C_CKCFGR_CCR, 4 << I2C_CKCFGR_CCR_Pos); //�էݧ� Fm mode. DUTY 1.

    //19.12.9 I2C Rise Time Register (I2Cx_RTR) (x=1/2)
    //MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 37 << I2C_RTR_TRISE_Pos);//�էݧ� Sm mode
    //MODIFY_REG(I2C1->RTR, I2C_RTR_TRISE, 12 << I2C_RTR_TRISE_Pos); //�էݧ� Fm mode

    SET_BIT(I2C1->CTLR1, I2C_CTLR1_PE); //I2C1 enable
}

/**
 *************************************************************************************
 *  @breif ����ߧܧ�ڧ� ��ܧѧߧڧ��ӧѧߧڧ� ������ۧ��ӧ� ��� �٧ѧէѧߧߧ�ާ� 7-�ҧڧ�ߧ�ާ� �ѧէ�֧��
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� true - �֧�ݧ� ������ۧ��ӧ� ��� �٧ѧէѧߧߧ�ާ� �ѧէ�֧�� ����٧ӧѧݧ���,
 *           false - �֧�ݧ� ������ۧ��ӧ� ��� �٧ѧէѧߧߧ�ާ� �ѧէ�֧�� �ߧ� ���ӧ֧�ѧ֧�
 *************************************************************************************
 */
bool RVMSIS_I2C_Adress_Device_Scan(I2C_TypeDef* I2C, uint8_t Adress_Device, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //������ѧӧݧ�֧� ��ڧԧߧѧ� START

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �էѧߧߧ�� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //���է�֧� + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧݧ�֧� ��ڧԧߧѧ� STOP
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;
        return true;
    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧݧ�֧� ��ڧԧߧѧ� STOP
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� ��֧�֧էѧ�� �էѧߧߧ�� ��� I2C
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @param  *data - ���ѧߧߧ��, �ܧ������ �ҧ�է֧� �����ѧӧݧ���
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� �ҧ�է֧� �����ѧӧݧ���.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� �����ѧӧܧ� �էѧߧߧ��. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Transmit(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //���է�֧� + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*������ѧӧڧ� �էѧߧߧ��*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //���ѧ�ڧ�� �ҧѧۧ��
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //���է֧�, ���ܧ� �էѧߧߧ�� �٧ѧԧ��٧���� �� ��֧ԧڧ��� ��էӧڧԧ�.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//�����ѧߧѧӧݧڧӧѧ֧�
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�

        return true;

    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� ���ڧ֧ާ� �էѧߧߧ�� ��� I2C
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @param  *data - ����է� �ҧ�է֧� �٧ѧ�ڧ��ӧѧ�� ���ڧߧ���� �էѧߧߧ��
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� �ҧ�է֧� ���ڧߧڧާѧ��.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� ���ڧ֧ާ� �էѧߧߧ��. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_I2C_Data_Receive(I2C_TypeDef* I2C, uint8_t Adress_Device, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1 | 1); //���է�֧� + �ܧ�ާѧߧէ� Read

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*�������֧� �էѧߧߧ��*/
        for (uint16_t i = 0; i < Size_data; i++) {
            if (i < Size_data - 1) {
                SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //����ݧ� �ާ� ����ڧ� ���ڧߧ��� ��ݧ֧է���ڧ� �ҧѧۧ�, ��� �����ѧӧݧ�֧� ACK

                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //���اڧէѧ֧�, ���ܧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ���� ����ӧ���� �էѧߧߧ��
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }

                *(data + i) = I2C->DATAR; //����֧ߧڧ� �ҧѧۧ��
            } else {
                CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //����ݧ� �ާ� �٧ߧѧ֧�, ���� ��ݧ֧է���ڧ� ���ڧߧ���� �ҧѧۧ� �ҧ�է֧� ����ݧ֧էߧڧ�, ��� �����ѧӧڧ� NACK

                SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
                Timeout_counter_ms = Timeout_ms;
                while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0) {
                    //���اڧէѧ֧�, ���ܧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ���� ����ӧ���� �էѧߧߧ��
                    if (!Timeout_counter_ms) {
                        return false;
                    }
                }
                *(data + i) = I2C->DATAR; //����֧ߧڧ� �ҧѧۧ��
            }
        }
        return true;

    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
        return false;
    }

}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� �٧ѧ�ڧ�� �� ��ѧާ��� ��� ��ܧѧ٧ѧߧߧ�ާ� �ѧէ�֧��
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @param  Adress_data - ���է�֧� �� ��ѧާ���, �ܧ�է� �ҧ�է֧� �٧ѧ�ڧ��ӧѧ�� �էѧߧߧ��
 *  @param  Size_adress - ���ѧ٧ާ֧� �ѧէ�֧�� �� �ҧѧۧ�ѧ�. ����ڧާ֧�: 1 - 8 �ҧڧ�ߧ�� �ѧէ�֧�. 2 - 16 �ҧڧ�ߧ�� �ѧէ�֧�.
 *  @param  *data - ���ѧߧߧ��, �ܧ������ �ҧ�է֧� �٧ѧ�ڧ��ӧѧ��
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� �ҧ�է֧� �٧ѧ�ڧ��ӧѧ��.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� �٧ѧ�ڧ��. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemWrite(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //���է�֧� + Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*������ѧӧڧ� �ѧէ�֧� ��ѧާ���*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //���ѧ�ڧ�� �ҧѧۧ��
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //���է֧�, ���ܧ� �էѧߧߧ�� �٧ѧԧ��٧���� �� ��֧ԧڧ��� ��էӧڧԧ�.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//�����ѧߧѧӧݧڧӧѧ֧�
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
                    return false;
                }
            }
        }

        /*����է֧� �٧ѧ�ڧ��ӧѧ�� �էѧߧߧ�� �� ���֧ۧܧ� ��ѧާ���, �ߧѧ�ڧߧѧ� �� ��ܧѧ٧ѧߧߧ�ԧ� �ѧէ�֧��*/
        for (uint16_t i = 0; i < Size_data; i++) {
            I2C->DATAR = *(data + i); //���ѧ�ڧ�� �ҧѧۧ��
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //���է֧�, ���ܧ� �էѧߧߧ�� �٧ѧԧ��٧���� �� ��֧ԧڧ��� ��էӧڧԧ�.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//�����ѧߧѧӧݧڧӧѧ֧�
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
                    return false;
                }
            }
        }

        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�

        return true;

    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF

        return false;
    }
}

/**
 **************************************************************************************************
 *  @breif ����ߧܧ�ڧ� ���֧ߧڧ� �ڧ� ��ѧާ��� ��� ��ܧѧ٧ѧߧߧ�ާ� �ѧէ�֧��
 *  @param  *I2C - ��ڧߧ� I2C
 *  @param  Adress_Device - ���է�֧� ������ۧ��ӧ�
 *  @param  Adress_data - ���է�֧� �� ��ѧާ���, ���ܧ�է� �ҧ�է֧� ���ڧ��ӧѧ�� �էѧߧߧ��
 *  @param  Size_adress - ���ѧ٧ާ֧� �ѧէ�֧�� �� �ҧѧۧ�ѧ�. ����ڧާ֧�: 1 - 8 �ҧڧ�ߧ�� �ѧէ�֧�. 2 - 16 �ҧڧ�ߧ�� �ѧէ�֧�.
 *  @param  *data - ���ѧߧߧ��, �� �ܧ������ �ҧ�է֧� �٧ѧ�ڧ��ӧѧ�� ���ڧ�ѧߧߧ�� �ڧߧ���ާѧ�ڧ�.
 *  @param  Size_data - ���ѧ٧ާ֧�, ��ܧ�ݧ�ܧ� �ҧѧۧ� �ҧ�է֧� ���ڧ��ӧѧ��.
 *  @retval  ����٧ӧ�ѧ�ѧ֧� ���ѧ��� ���ڧ��ӧѧߧڧ�. True - �����֧�ߧ�. False - ����ڧҧܧ�.
 **************************************************************************************************
 */
bool RVMSIS_I2C_MemRead(I2C_TypeDef* I2C, uint8_t Adress_Device, uint16_t Adress_data, uint8_t Size_adress, uint8_t* data, uint16_t Size_data, uint32_t Timeout_ms) {

    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/
    if (READ_BIT(I2C->STAR2, I2C_STAR2_BUSY)) {
        //����ݧ� ��ڧߧ� �٧ѧߧ���

        if ((READ_BIT(GPIOC->INDR, GPIO_INDR_IDR1)) && (READ_BIT(GPIOC->INDR, GPIO_INDR_IDR2))) {
            //����ݧ� �ݧڧߧڧ� �ߧ� ��ѧާ�� �է֧ݧ� ��ӧ�ҧ�էߧ�, �� BUSY �ӧڧ�ڧ�
            RVMSIS_I2C_Reset(); //��֧�֧�
            RVMSIS_I2C1_Init(); //���ӧ���ߧѧ� �ڧߧڧ�ڧѧݧڧ٧ѧ�ڧ�
        }

        if (READ_BIT(I2C->STAR2, I2C_STAR2_MSL)) {
            //����ݧ� ����ڧ� ���ѧ���, ���� �ާ� �� �ާѧ��֧��
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //������ѧӧڧ� ��ڧԧߧѧ� STOP
        }

        if (I2C->CTLR1 != 1) {
            //����ݧ� �� CR1 ����-��� �ݧڧ�ߧ֧�, ��� ��֧�֧٧ѧԧ��٧ڧ� I2C
            CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_PE);
            SET_BIT(I2C->CTLR1, I2C_CTLR1_PE);
        }

        return false;
    }
    /*-------------------�����ӧ֧�ܧ� �٧ѧߧ������ ��ڧߧ�-------------------*/

    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_POS); //���ڧ� ACK ����ѧӧݧ�֧� (N)ACK ��֧ܧ��֧ԧ� �ҧѧۧ��, ���ڧߧڧާѧ֧ާ�ԧ� �� ��էӧڧԧ�ӧ�� ��֧ԧڧ����.
    SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

    Timeout_counter_ms = Timeout_ms;
    while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
        //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

        if (!Timeout_counter_ms) {
            return false;
        }

    }
    //����������������!
    /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
    I2C->STAR1;
    I2C->DATAR = (Adress_Device << 1); //���է�֧� + �ܧ�ާѧߧէ� Write

    Timeout_counter_ms = Timeout_ms;
    while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
        //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

        if (!Timeout_counter_ms) {
            return false;
        }

    }

    if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
        //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
        /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
        I2C->STAR1;
        I2C->STAR2;

        /*������ѧӧڧ� �ѧէ�֧� ��ѧާ���*/
        for (uint16_t i = 0; i < Size_adress; i++) {
            I2C->DATAR = *((uint8_t*) &Adress_data + (Size_adress - 1 - i)); //���ѧ�ڧ�� �ҧѧۧ��
            while (READ_BIT(I2C->STAR1, I2C_STAR1_TXE) == 0) {
                //���է֧�, ���ܧ� �էѧߧߧ�� �٧ѧԧ��٧���� �� ��֧ԧڧ��� ��էӧڧԧ�.

                if ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 1)) {
                    //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP);//�����ѧߧѧӧݧڧӧѧ֧�
                    CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF);//���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
                    return false;
                }
            }
        }

        //����ӧ���ߧ�� ���ѧ��
        SET_BIT(I2C->CTLR1, I2C_CTLR1_START); //����ѧ���֧�.

        Timeout_counter_ms = Timeout_ms;
        while (READ_BIT(I2C->STAR1, I2C_STAR1_SB) == 0) {
            //���اڧէѧ֧� �է� �ާ�ާ֧ߧ��, ���ܧ� �ߧ� ���ѧҧ��ѧ֧� Start condition generated

            if (!Timeout_counter_ms) {
                return false;
            }

        }
        //����������������!
        /* ���ڧ� I2C_SR1_SB ���ڧ�ѧ֧��� ����ԧ�ѧާާߧ� ����֧� ���֧ߧڧ� ��֧ԧڧ���� SR1 �� ����ݧ֧է���֧� �٧ѧ�ڧ��� �� ��֧ԧڧ��� DR �ڧݧ� �ܧ�ԧէ� PE=0*/
        I2C->STAR1;
        I2C->DATAR = (Adress_Device << 1 | 1); //���է�֧� + �ܧ�ާѧߧէ� Read

        Timeout_counter_ms = Timeout_ms;
        while ((READ_BIT(I2C->STAR1, I2C_STAR1_AF) == 0) && (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR) == 0)) {
            //���է֧�, ���ܧ� �ѧէ�֧� ���٧�ӧ֧���

            if (!Timeout_counter_ms) {
                return false;
            }

        }

        if (READ_BIT(I2C->STAR1, I2C_STAR1_ADDR)) {
            //����ݧ� ������ۧ��ӧ� ����٧ӧѧݧ���, ��ҧ���ڧ� �ҧڧ� ADDR
            /*���ҧ��� �ҧڧ�� ADDR ����ڧ٧ӧ�էڧ��� ���֧ߧڧ֧� SR1, �� ������ SR2*/
            I2C->STAR1;
            I2C->STAR2;

            /*�������֧� �էѧߧߧ��, �ߧѧ�ڧߧѧ� �� ��ܧѧ٧ѧߧߧ�ԧ� �ѧէ�֧��*/
            for (uint16_t i = 0; i < Size_data; i++) {
                if (i < Size_data - 1) {
                    SET_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //����ݧ� �ާ� ����ڧ� ���ڧߧ��� ��ݧ֧է���ڧ� �ҧѧۧ�, ��� �����ѧӧݧ�֧� ACK
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    *(data + i) = I2C->DATAR; //����֧ߧڧ� �ҧѧۧ��
                } else {
                    CLEAR_BIT(I2C->CTLR1, I2C_CTLR1_ACK); //����ݧ� �ާ� �٧ߧѧ֧�, ���� ��ݧ֧է���ڧ� ���ڧߧ���� �ҧѧۧ� �ҧ�է֧� ����ݧ֧էߧڧ�, ��� �����ѧӧڧ� NACK

                    SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
                    while (READ_BIT(I2C->STAR1, I2C_STAR1_RXNE) == 0);
                    //����է�اէ֧�, ���ܧ� ��էӧڧԧ�ӧ�� ��֧ԧڧ��� �����ݧߧڧ��� �ߧ�ӧ�� �ҧѧۧ��� �էѧߧߧ��
                    *(data + i) = I2C->DATAR; //����֧ߧڧ� �ҧѧۧ��
                }
            }
            return true;

        } else {
            //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
            SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
            CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
            return false;
        }

    } else {
        //����ݧ� ������ۧ��ӧ� �ߧ� ����٧ӧѧݧ���, ���ڧݧ֧�ڧ� 1 �� I2C_SR1_AF
        SET_BIT(I2C->CTLR1, I2C_CTLR1_STOP); //�����ѧߧѧӧݧڧӧѧ֧�
        CLEAR_BIT(I2C->STAR1, I2C_STAR1_AF); //���ҧ�ѧ��ӧѧ֧� �ҧڧ� AF
        return false;
    }
}


