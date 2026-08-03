#ifndef TIM3_H
#define TIM3_H

#include "ch32f20x.h"
#include "ch32f20x_rcc.h"
#include "ch32f20x_tim.h"

#define PWM_BASE_CLK		20000UL

#define SETPWM_1(x)			TIM_SetCompare1(TIM3, x)
#define SETPWM_2(x)			TIM_SetCompare2(TIM3, x)

void vTim3_initPWMTimer(void);
void vTim3_stopPWMTimer(void);
void vTim3_startPWMTimer(void);

#endif