#ifndef SYSTICK_H
#define SYSTICK_H

#include <stm32f303xc.h>

#define TO_MSEC(x) (x * 10U)

void systick_init();
uint32_t millis();

#endif