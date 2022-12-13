/*
 * timer.h
 *
 *  Created on: 1 dic. 2022
 *      Author: franb
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "main.h"

void inic_timer_3(uint32_t divisor_us);
void start_timer_3(void);
uint32_t valor_timer_3(void);
void stop_timer_3(void);
void inic_timer_4(uint32_t divisor_us);
void start_timer_4(void);
uint32_t valor_timer_4(void);
void stop_timer_4(void);

#endif /* INC_TIMER_H_ */
