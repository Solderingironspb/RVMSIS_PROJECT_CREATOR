#include "main.h"

int main(void) {
    RVMSIS_Debug_init(); //Настройка дебага
    RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты
    RVMSIS_SysTick_Timer_init(); //Настройка системного таймера

    while(1) {

    }
}
