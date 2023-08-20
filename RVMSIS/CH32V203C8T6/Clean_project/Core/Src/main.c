#include "main.h"
#include "stdio.h"

#define DEBUG_USE   //Использовать DEBUG по USART

int main(void) {
    RVMSIS_Debug_init(); //Настройка дебага
    RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты
    RVMSIS_SysTick_Timer_init(); //Настройка системного таймера
    RVMSIS_USART3_Init(); //См. файл syscalls.c USART3 115200 8N1 выбран для отладки через printf. Ножка PB10
#ifdef DEBUG_USE
    printf("Hello world!\r\n");
#endif

    while(1) {

    }
}
