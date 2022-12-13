/*
 * timer.c
 *
 *  Created on: 1 dic. 2022
 *      Author: franb
 */
#include "timer.h"

//funciones timer
void inic_timer_3(uint32_t divisor_us) // si paso 1, cuenta cada 1 useg.
{
	/*
	 * Uso el timer 3 para la maquina de estados.
	 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	/*
	 * Esta línea configura el prescaler del timer
	 * que cuenta tiempo del procesador. Prestar atención.
	 */
	TIM3->PSC = (SystemCoreClock / (1000000*divisor_us)) - 1;
	TIM3->CNT = -1;
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 &= ~TIM_CR1_CEN;
}

void start_timer_3(void)
{
	TIM3->CNT = 0;
	TIM3->CR1 |= TIM_CR1_CEN;
}

uint32_t valor_timer_3(void)
{
	uint32_t ret = TIM3->CNT;//devuelve el conteo

	return ret;
}

void stop_timer_3(void)
{
	TIM3->CR1 &= ~TIM_CR1_CEN;
}

void inic_timer_4(uint32_t divisor_us) // si paso 1, cuenta cada 1 useg.
{
	/*
	 * Uso el timer 4 para la diferencia de tiempo.
	 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	/*
	 * Esta línea configura el prescaler del timer
	 * que cuenta tiempo del procesador. Prestar atención.
	 */
	TIM4->PSC = (SystemCoreClock / (1000000*divisor_us)) - 1;
	TIM4->CNT = -1;
	TIM4->CR1 |= TIM_CR1_CEN;
	TIM4->CR1 &= ~TIM_CR1_CEN;
}

void start_timer_4(void)
{
	TIM4->CNT = 0;
	TIM4->CR1 |= TIM_CR1_CEN;
}

uint32_t valor_timer_4(void)
{
	uint32_t ret = TIM4->CNT;//devuelve el conteo

	return ret;
}

void stop_timer_4(void)
{
	TIM4->CR1 &= ~TIM_CR1_CEN;
}
