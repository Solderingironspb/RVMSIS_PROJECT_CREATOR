/*
 * ch32v003_it.h
 *
 *  Created on: May 16, 2024
 *      Author: Solderingiron
 */

#ifndef INC_CH32V003XX_IT_H_
#define INC_CH32V003XX_IT_H_


#include "main.h"
#include "ch32v00x_RVMSIS.h"



__attribute__((interrupt("machine"))) void SysTick_Handler(void);



#endif /* INC_CH32V003XX_IT_H_ */
