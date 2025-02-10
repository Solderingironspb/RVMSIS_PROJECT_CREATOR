#include "main.h"

int main(void) {
    RVMSIS_RCC_SystemClock_48MHz();
    RVMSIS_SysTick_Timer_init();
    RVMSIS_GPIO_init(GPIOC, 2, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_50_MHZ);

    while(1) {
        GPIOC->BSHR = GPIO_BSHR_BS2;
        Delay_ms(100);
        GPIOC->BSHR = GPIO_BSHR_BR2;
        Delay_ms(100);

    }
}
