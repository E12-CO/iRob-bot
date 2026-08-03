#include "tim5.h"

void vTim5_initEthernetRunnerInt(void){
	// Enable TIM5 clock 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	TIM_DeInit(TIM5);
	
	TIM_PrescalerConfig(
		TIM5, 
		SYSCLK_FREQ_120MHz_HSE / 10000UL, 
		TIM_PSCReloadMode_Immediate
	);
	TIM_CounterModeConfig(TIM5, TIM_CounterMode_Up);
	TIM_SetAutoreload(TIM5, ((WCHNETTIMERPERIOD * 10000UL) / 100) - 1);
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_UpdateRequestConfig(TIM5, TIM_UpdateSource_Regular);
	
	// Enabling the TIM5 interrupt 
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_ClearFlag(TIM5, 
		TIM_IT_Update	
	);
	NVIC_SetPriority(TIM5_IRQn, 3);
	NVIC_EnableIRQ(TIM5_IRQn);
	TIM_Cmd(TIM5, ENABLE);
}