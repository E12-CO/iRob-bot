#ifndef TIM4_H
#define TIM4_H

#include "ch32f20x.h"
#include "ch32f20x_rcc.h"
#include "ch32f20x_tim.h"

#define GET_ENC_CNT		TIM_GetCounter(TIM4)

void vTim4_initEncoder(void);

#endif