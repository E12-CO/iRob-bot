// 12 Bit 20kHz PWM for motor control
#include "tim_pwm.h"

void tim_pwm_init(){
	// Enable TIM1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	
	// Init TIM1 in PWM mode and enable 4 channels
	TIM1->ARR = 4095;// 12-bit timer
	TIM1->PSC	= (3600 - 1);// 72Mhz/20kHz = 3600
	
	TIM1->EGR |= 
		TIM_EGR_UG; // Update generation
	
	// config Output compare on Channel 1 and 2
	TIM1->CCMR1 |= 
		(3 << TIM_CCMR1_OC2M_Pos)	|
		TIM_CCMR1_OC2PE						|
		(3 << TIM_CCMR1_OC1M_Pos)	|
		TIM_CCMR1_OC1PE						;
	
	// config output compare on Channel 3 and 4
	TIM1->CCMR2 |= 
		(3 << TIM_CCMR2_OC4M_Pos)	|
		TIM_CCMR2_OC4PE						|
		(3 << TIM_CCMR2_OC3M_Pos)	|
		TIM_CCMR2_OC3PE						;
	
	// Enable output compare
	TIM1->CCER |=
		TIM_CCER_CC4E		|
		TIM_CCER_CC3E		|
		TIM_CCER_CC2E		|
		TIM_CCER_CC1E		;
	
	// Enable counter
	TIM1->CR1 |= TIM_CR1_CEN;
}

void tim_pwm_updateCompare(
	motor_duty_t *duty_ptr_t
	){
	
	TIM1->CCR1 = duty_ptr_t->m1;
	TIM1->CCR2 = duty_ptr_t->m2;
	TIM1->CCR3 = duty_ptr_t->m3;
	TIM1->CCR4 = duty_ptr_t->m4;
}