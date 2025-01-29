#include "tim_qdec.h"

void tim_qdec_init(){
	// Enable clock for Timers
	// We will be using Timer8, Timer2, Timer3 and Timer 4 to count the encoder
	RCC->APB2ENR |= 
		RCC_APB2ENR_TIM8EN;
	RCC->APB1ENR |=
		RCC_APB1ENR_TIM2EN	|
		RCC_APB1ENR_TIM3EN	|
		RCC_APB1ENR_TIM4EN	;
	
	// Init TIM2
	TIM2->CCMR1 |=
		TIM_CCMR1_CC1S_0		|		
		TIM_CCMR1_CC2S_0		;		
	
	TIM2->SMCR |= 
		TIM_SMCR_SMS_0	|
		TIM_SMCR_SMS_1	;
	
	TIM2->ARR = 0xFFFF;
	
	// Init TIM3
	TIM3->CCMR1 |=
		TIM_CCMR1_CC1S_0		|		
		TIM_CCMR1_CC2S_0		;		
	
	TIM3->SMCR |= 
		TIM_SMCR_SMS_0	|
		TIM_SMCR_SMS_1	;
	
	TIM3->ARR = 0xFFFF;
	
	// Init TIM4
	TIM4->CCMR1 |=
		TIM_CCMR1_CC1S_0		|		
		TIM_CCMR1_CC2S_0		;		
	
	TIM4->SMCR |= 
		TIM_SMCR_SMS_0	|
		TIM_SMCR_SMS_1	;
	
	TIM4->ARR = 0xFFFF;
	
	// Init TIM8
	TIM8->CCMR1 |=
		TIM_CCMR1_CC1S_0		|		
		TIM_CCMR1_CC2S_0		;		
	
	TIM8->SMCR |= 
		TIM_SMCR_SMS_0	|
		TIM_SMCR_SMS_1	;
	
	TIM8->ARR = 0xFFFF;
	
	// Turn all timer
	TIM2->CR1		|= TIM_CR1_CEN;
	TIM3->CR1		|= TIM_CR1_CEN;
	TIM4->CR1		|= TIM_CR1_CEN;
	TIM8->CR1		|= TIM_CR1_CEN;
	
}

void tim_qdec_getCount(
	encoder_cnt_t *cnt_ptr
	){
	cnt_ptr->enc1 = TIM3->CNT;
	cnt_ptr->enc2 = (uint16_t)TIM2->CNT;
	cnt_ptr->enc3	= TIM8->CNT;
	cnt_ptr->enc4 = TIM4->CNT;
}