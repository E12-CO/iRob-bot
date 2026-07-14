#ifndef SYSTICK_H
#define SYSTICK_H

#include "ch32f20x.h"
#include "eth_driver.h"

void vSystick_init(
	const uint32_t f_cpu, 
	const uint32_t Hz);
uint32_t u32Millis();
void vDelay_ms(uint32_t ms);

#endif