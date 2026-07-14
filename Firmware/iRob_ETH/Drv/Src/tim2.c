#include "tim2.h"

void vTim2_initLoopTimer(void){
	TIM_TimeBaseInitTypeDef tTim2InitStruct = { 0 };
	
	// Enable TIM2 clock 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// Set up the prescaler and leave the ARR out for now
	tTim2InitStruct.TIM_Prescaler 	= SYSCLK_FREQ_120MHz_HSE / LOOP_BASE_CLK;// Divide 120MHz by 10k to get the value for 10kHz clock
	tTim2InitStruct.TIM_Period		= 0;
	tTim2InitStruct.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &tTim2InitStruct);
	
	// Enabling the TIM5 interrupt 
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0);
}

void vTim2_stopLoopTimer(void){
	TIM_Cmd(TIM2, DISABLE);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	TIM_SetCounter(TIM2, 0);
}

void vTim2_startLoopTimer(void){
	TIM_SetCounter(TIM2, 0);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	TIM_Cmd(TIM2, ENABLE);
}

void vTim2_setLoopRate(uint16_t u16LoopRateARR){
	TIM_SetAutoreload(TIM2, u16LoopRateARR);
}