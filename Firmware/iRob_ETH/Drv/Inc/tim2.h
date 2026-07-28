#ifndef TIM2_H
#define TIM2_H

#include "ch32f20x.h"
#include "ch32f20x_rcc.h"
#include "ch32f20x_tim.h"

#define LOOP_BASE_CLK		10000UL

#define LOOP_RATE_2KHZ		(LOOP_BASE_CLK/2000)

void vTim2_initLoopTimer(void);
void vTim2_stopLoopTimer(void);
void vTim2_startLoopTimer(void);

#endif