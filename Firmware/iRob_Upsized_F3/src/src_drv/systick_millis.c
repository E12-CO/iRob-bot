#include "systick_millis.h"

#define SYS_FCPU			72000000UL

// Private variables
volatile uint32_t millis_counter;

void SysTick_Handler(void){
	millis_counter++;
}

void systick_init(){
	SysTick->LOAD = (SYS_FCPU / 10000) - 1;// Generate 10kHz interrupt
	SysTick->VAL = 0;
	SysTick->CTRL |= 
		(1 << SysTick_CTRL_CLKSOURCE_Pos)	|
		(1 << SysTick_CTRL_TICKINT_Pos)		;
	
	SysTick->CTRL |= 
		(1 << SysTick_CTRL_ENABLE_Pos);
	
	NVIC_SetPriority(SysTick_IRQn, 0);
	NVIC_ClearPendingIRQ(SysTick_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
}

uint32_t millis(){
	return millis_counter;
}