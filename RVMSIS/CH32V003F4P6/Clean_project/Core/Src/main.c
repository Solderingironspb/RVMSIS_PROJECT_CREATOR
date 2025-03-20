#include "main.h"

uint32_t Counter = 0;

int main(void) {
    RVMSIS_RCC_SystemClock_48MHz();  // Настройка RCC на 48 МГц
    RVMSIS_SysTick_Timer_init();     // Настройка системного таймера
    RVMSIS_USART1_Init();

    // Настроим GPIOD на выход: GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_30_MHZ
    SET_BIT(RCC->APB2PCENR, RCC_APB2Periph_GPIOD);                             // Включим тактирование порта D
    MODIFY_REG(GPIOD->CFGLR, GPIO_CFGLR_CNF4, 0b00 << GPIO_CFGLR_CNF4_Pos);    // Universal push-pull output mode.
    MODIFY_REG(GPIOD->CFGLR, GPIO_CFGLR_MODE4, 0b11 << GPIO_CFGLR_MODE4_Pos);  // Output mode, maximum speed 30MHz.

    while (1) {
        GPIOD->BSHR = GPIO_BSHR_BR4;
        Delay_ms(100);
        GPIOD->BSHR = GPIO_BSHR_BS4;
        Delay_ms(100);
        Counter = Counter + 32;
        RVMSIS_USART_Transmit(USART1, (uint8_t*)&Counter, sizeof(Counter), 100);
    }
}
