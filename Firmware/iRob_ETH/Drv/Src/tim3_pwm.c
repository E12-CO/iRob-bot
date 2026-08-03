#include "tim3.h"

void vTim3_initPWMTimer(void){
	TIM_TimeBaseInitTypeDef tTim3InitStruct = { 0 };
	TIM_OCInitTypeDef	tTim3PwmInitStruct = {0};
	
	// Enable TIM2 clock 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Set up the prescaler and set ARR to 4095 for 12-bit PWM
	tTim3InitStruct.TIM_Prescaler 	= SYSCLK_FREQ_120MHz_HSE / PWM_BASE_CLK;
	tTim3InitStruct.TIM_Period		= 4096 - 1;
	tTim3InitStruct.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &tTim3InitStruct);
	
	tTim3PwmInitStruct.TIM_OCMode		= TIM_OCMode_PWM1;
	tTim3PwmInitStruct.TIM_OCPolarity	= TIM_OCPolarity_Low;
	tTim3PwmInitStruct.TIM_Pulse		= 0x0000;
	
	TIM_OC1Init(TIM3, &tTim3PwmInitStruct);
	TIM_OC1Init(TIM3, &tTim3PwmInitStruct);
	
	TIM_SetCompare1(TIM3, 0);
	TIM_SetCompare2(TIM3, 0);

}

void vTim3_stopPWMTimer(void){
	TIM_Cmd(TIM3, DISABLE);
	TIM_SetCounter(TIM3, 0);
}

void vTim3_startPWMTimer(void){
	TIM_SetCounter(TIM3, 0);
	TIM_Cmd(TIM3, ENABLE);
}
