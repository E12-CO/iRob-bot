#include "systick.h"

volatile uint32_t u32MillisCounter;

void SysTick_Handler(void){
	u32MillisCounter++;
	if((u32MillisCounter%100) == 0)
		WCHNET_TimeIsr(WCHNETTIMERPERIOD);
}

void vSystick_init(const uint32_t f_cpu, const uint32_t Hz){
	SysTick->LOAD = (f_cpu / Hz) - 1;
	SysTick->VAL	=	0;
	SysTick->CTRL |= 
			(1 << SysTick_CTRL_CLKSOURCE_Pos) |
			(1 << SysTick_CTRL_TICKINT_Pos);
	SysTick->CTRL |= (1 << SysTick_CTRL_ENABLE_Pos);
	NVIC_SetPriority(SysTick_IRQn, 0);
	NVIC_ClearPendingIRQ(SysTick_IRQn);
	NVIC_EnableIRQ(SysTick_IRQn);
}
	
uint32_t millis(){
	return u32MillisCounter;
}

void delay_ms(uint32_t ms){
	static uint32_t  elapsed;
	if(ms == 0)
		return;
	
	elapsed = millis();
	while((millis()) - elapsed < ms);
}