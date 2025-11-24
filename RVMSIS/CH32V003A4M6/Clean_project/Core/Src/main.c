#include "main.h"

int main(void) {
    RVMSIS_RCC_SystemClock_48MHz();  // Настройка RCC на 48 МГц
    RVMSIS_SysTick_Timer_init();     // Настройка системного таймера

    while (1) {
    }
}
