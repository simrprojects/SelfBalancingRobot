/*
 * delay.h
 *
 *  Created on: 27.08.2017
 *      Author: Mariusz
 */

#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f7xx.h"

void SysTick_Init(void);
void TimeTick_Decrement(void);
void delay_nus(int n);
void delay_1ms(void);
void delay_ms(int n);
int get_ms(unsigned long *count);





#endif /* DELAY_H_ */
