#ifndef TIM2_H
#define TIM2_H

#include "ch32f20x.h"
#include "ch32f20x_rcc.h"
#include "ch32f20x_tim.h"

#define LOOP_BASE_CLK		10000UL

#define LOOP_RATE_2KHZ		(LOOP_BASE_CLK/2000)
#define LOOP_RATE_1KHZ		(LOOP_BASE_CLK/1000)
#define LOOP_RATE_500HZ		(LOOP_BASE_CLK/500)
#define LOOP_RATE_250HZ		(LOOP_BASE_CLK/250)
#define LOOP_RATE_100HZ		(LOOP_BASE_CLK/100)
#define LOOP_RATE_50HZ		(LOOP_BASE_CLK/50)

void vTim2_initLoopTimer(void);
void vTim2_stopLoopTimer(void);
void vTim2_startLoopTimer(void);
void vTim2_setLoopRate(uint16_t u16LoopRateARR);


#endif