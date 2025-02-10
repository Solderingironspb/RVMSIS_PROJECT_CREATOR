/*
 * ch32v003_it.c
 *
 *  Created on: May 16, 2024
 *      Author: Solderingiron
 */

#include "main.h"
#include "ch32v003xx_it.h"
//#include <math.h>

void TIM1_UP_IRQHandler(void) {
    if (READ_BIT(TIM1->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM1->INTFR, TIM_UIF); //���ҧ���ڧ� ��ݧѧ� ���֧���ӧѧߧڧ�
    }
}

void TIM2_IRQHandler(void) {
    if (READ_BIT(TIM2->INTFR, TIM_UIF)) {
        CLEAR_BIT(TIM2->INTFR, TIM_UIF); //���ҧ���ڧ� ��ݧѧ� ���֧���ӧѧߧڧ�
    }
}

void ADC1_2_IRQHandler(void) {
    if (READ_BIT(ADC1->STATR, ADC_EOC)) {
        ADC1->IDATAR1; //���ڧ�ѧ֧� �ܧѧߧѧ�, ����� ��ҧ���ڧ�� ��ݧѧ�
    }

}

extern volatile uint16_t ADC_RAW_Data[2];

const uint16_t Arr[1024] = { 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9,
        9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13,
        13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 18,
        18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24,
        25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31, 32, 32, 32, 32, 32, 33, 33, 33, 33, 34, 34,
        34, 34, 35, 35, 35, 35, 36, 36, 36, 36, 37, 37, 37, 37, 38, 38, 38, 38, 39, 39, 39, 39, 40, 40, 40, 41, 41, 41, 41, 42, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 45, 45, 46, 46, 46, 47, 47,
        47, 48, 48, 48, 49, 49, 49, 50, 50, 50, 51, 51, 51, 52, 52, 52, 53, 53, 54, 54, 54, 55, 55, 55, 56, 56, 56, 57, 57, 58, 58, 58, 59, 59, 60, 60, 60, 61, 61, 62, 62, 63, 63, 63, 64, 64, 65, 65,
        66, 66, 67, 67, 67, 68, 68, 69, 69, 70, 70, 71, 71, 72, 72, 73, 73, 74, 74, 75, 75, 76, 76, 77, 77, 78, 78, 79, 79, 80, 80, 81, 81, 82, 83, 83, 84, 84, 85, 85, 86, 87, 87, 88, 88, 89, 90, 90,
        91, 92, 92, 93, 93, 94, 95, 95, 96, 97, 97, 98, 99, 99, 100, 101, 101, 102, 103, 103, 104, 105, 105, 106, 107, 108, 108, 109, 110, 111, 111, 112, 113, 114, 114, 115, 116, 117, 117, 118, 119,
        120, 121, 122, 122, 123, 124, 125, 126, 127, 127, 128, 129, 130, 131, 132, 133, 134, 135, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154,
        155, 156, 157, 158, 159, 160, 161, 163, 164, 165, 166, 167, 168, 170, 171, 172, 173, 174, 175, 176, 178, 179, 180, 181, 182, 184, 185, 186, 187, 189, 190, 191, 193, 194, 195, 197, 198, 199,
        200, 202, 203, 204, 206, 207, 209, 210, 212, 213, 215, 216, 217, 219, 220, 222, 224, 225, 226, 228, 230, 231, 232, 234, 236, 237, 239, 241, 242, 244, 246, 247, 249, 251, 252, 254, 256, 257,
        259, 261, 263, 265, 266, 268, 270, 272, 274, 275, 277, 279, 281, 283, 285, 287, 289, 291, 293, 295, 297, 299, 301, 303, 305, 307, 309, 311, 313, 315, 317, 319, 322, 324, 326, 329, 331, 333,
        335, 337, 339, 342, 344, 346, 349, 351, 354, 356, 359, 361, 363, 366, 368, 370, 373, 376, 378, 381, 383, 386, 389, 391, 394, 397, 399, 402, 405, 408, 410, 413, 416, 419, 421, 425, 427, 430,
        433, 436, 439, 442, 445, 448, 451, 454, 457, 461, 463, 466, 470, 473, 476, 479, 483, 486, 488, 492, 495, 499, 503, 506, 509, 513, 516, 519, 523, 527, 530, 534, 537, 541, 545, 549, 552, 555,
        560, 563, 567, 571, 575, 578, 583, 587, 590, 595, 599, 602, 607, 611, 615, 620, 623, 627, 631, 636, 640, 644, 649, 653, 657, 663, 667, 671, 676, 680, 685, 690, 694, 699, 704, 709, 713, 719,
        723, 728, 732, 738, 743, 747, 753, 758, 763, 769, 773, 778, 785, 789, 794, 801, 806, 810, 817, 822, 827, 834, 839, 844, 849, 856, 861, 867, 874, 879, 884, 892, 897, 903, 910, 916, 921, 929,
        934, 940, 948, 954, 959, 967, 973, 979, 985, 993, 999, 1005, 1014, 1020, 1026, 1034, 1041, 1047, 1056, 1062, 1069, 1077, 1084, 1090, 1099, 1106, 1113, 1122, 1129, 1136, 1143, 1152, 1159, 1166,
        1176, 1183, 1190, 1200, 1207, 1215, 1224, 1232, 1239, 1250, 1257, 1265, 1275, 1283, 1291, 1299, 1309, 1317, 1325, 1336, 1344, 1353, 1364, 1372, 1380, 1392, 1400, 1409, 1420, 1429, 1438, 1449,
        1458, 1467, 1479, 1488, 1497, 1506, 1519, 1528, 1537, 1550, 1559, 1569, 1582, 1591, 1601, 1614, 1624, 1634, 1647, 1657, 1668, 1681, 1692, 1702, 1716, 1726, 1737, 1747, 1762, 1772, 1783, 1798,
        1809, 1820, 1835, 1846, 1857, 1872, 1884, 1895, 1911, 1923, 1934, 1950, 1962, 1974, 1990, 2002, 2015, 2027, 2043, 2056, 2068, 2085, 2098, 2111, 2128, 2141, 2154, 2172, 2185, 2198, 2216, 2230,
        2244, 2262, 2276, 2290, 2304, 2322, 2337, 2351, 2370, 2385, 2399, 2419, 2434, 2448, 2468, 2484, 2499, 2519, 2535, 2550, 2571, 2587, 2602, 2624, 2640, 2656, 2672, 2694, 2710, 2727, 2749, 2766,
        2783, 2806, 2823, 2840, 2863, 2881, 2898, 2922, 2940, 2958, 2982, 3000, 3018, 3043, 3062, 3080, 3099, 3125, 3144, 3163, 3189, 3208, 3228, 3254, 3274, 3294, 3321, 3341, 3362, 3389, 3410, 3431,
        3459, 3480, 3501, 3530, 3551, 3573, 3595, 3624, 3646, 3669, 3699, 3721, 3744, 3774, 3798, 3821, 3852, 3876, 3899, 3931, 3955, 3979, 4012, 4036, 4061, 4094 };

void DMA1_Channel1_IRQHandler(void) {
    if (READ_BIT(DMA1->INTFR, DMA_TCIF1)) {
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //���ҧ���ڧ� �ԧݧ�ҧѧݧ�ߧ��� ��ݧѧ�.
        // Lin = 3.02444f * ADC_RAW_Data[0] + 1000;
        TIM1->CH2CVR = Arr[ADC_RAW_Data[0]];       //exp(Lin / 492.219971f);

        /*���է֧�� �ާ�اߧ� ��ڧ�ѧ�� �ܧ��*/

    } else if (READ_BIT(DMA1->INTFR, DMA_TEIF1)) {
        /*���է֧�� �ާ�اߧ� ��է֧ݧѧ�� �ܧѧܧ��-��� ��ҧ�ѧҧ���ڧ� ���ڧҧ��*/
        SET_BIT(DMA1->INTFCR, DMA_CGIF1); //���ҧ���ڧ� �ԧݧ�ҧѧݧ�ߧ��� ��ݧѧ�.
    }
}

/**
 ***************************************************************************************
 *  @breif ���ѧ����ۧܧ� Delay �� �ѧߧѧݧ�� HAL_GetTick()
 ***************************************************************************************
 */
extern volatile uint32_t SysTimer_ms; //���֧�֧ާ֧ߧߧѧ�, �ѧߧѧݧ�ԧڧ�ߧѧ� HAL_GetTick()
extern volatile uint32_t Delay_counter_ms; //����֧��ڧ� �էݧ� ���ߧܧ�ڧ� Delay_ms
extern volatile uint32_t Timeout_counter_ms; //���֧�֧ާ֧ߧߧѧ� �էݧ� ��ѧۧާѧ��� ���ߧܧ�ڧ�

/**
 ******************************************************************************
 *  @breif ����֧���ӧѧߧڧ� ��� ��ݧѧԧ� CNTIF
 ******************************************************************************
 */
void SysTick_Handler(void) {
    SysTick->SR &= ~(1 << 0);
    SysTimer_ms++;

    if (Delay_counter_ms) {
        Delay_counter_ms--;
    }
    if (Timeout_counter_ms) {
        Timeout_counter_ms--;
    }
}
