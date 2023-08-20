#include "main.h"
#include "stdio.h"

#define DEBUG_USE   //Использовать DEBUG по USART
uint8_t Priority = 0; //Попробуем узнать приоритет прерывания
uint32_t Counter = 0;

//Создадим структуру для теста
typedef struct
    __attribute__((packed)) {
        uint8_t Data1;
        uint16_t Data2;
        uint32_t Data3;
        float Data4;
    } Flash_struct;
    Flash_struct Flash_data_CH32;
    Flash_struct Flash_data_CH32_read;

    int main(void) {
        RVMSIS_Debug_init(); //Настройка дебага
        RVMSIS_RCC_SystemClock_144MHz(); //Настройка системной частоты
        RVMSIS_SysTick_Timer_init(); //Настройка системного таймера
        RVMSIS_USART3_Init(); //См. файл syscalls.c USART3 115200 8N1 выбран для отладки через printf. Ножка PB10
        RVMSIS_GPIO_init(GPIOC, 13, GPIO_GENERAL_PURPOSE_OUTPUT, GPIO_OUTPUT_PUSH_PULL, GPIO_SPEED_50_MHZ); //Настройка PC13 в режим Push-pull
        /*Заполним структуру данных*/
        Flash_data_CH32.Data1 = 255;
        Flash_data_CH32.Data2 = 0x4567;
        Flash_data_CH32.Data3 = 0x89101112;
        Flash_data_CH32.Data4 = 3.14159f;
        //Попробуем записать ее во Flash
        RVMSIS_FLASH_Page_write(0x0800F000, (uint8_t*) &Flash_data_CH32, sizeof(Flash_data_CH32));
        //Теперь попробуем считать ее из Flash
        RVMSIS_FLASH_Read_data(0x0800F000, (uint8_t*) &Flash_data_CH32_read, sizeof(Flash_data_CH32_read));
        //Узнаем приоритет прерывания SysTick_IRQn
        Priority = NVIC_GetPriority(SysTicK_IRQn);

#ifdef DEBUG_USE
        printf("Hello world!\r\n");
#endif

        while(1) {

#ifdef DEBUG_USE
            printf("Счетчик равен = %d\r\n", Counter);
            Counter++;
#endif
            GPIOC->BSHR = GPIO_BSHR_BS13;
            Delay_ms(100);
            GPIOC->BSHR = GPIO_BSHR_BR13;
            Delay_ms(100);
        }
    }
