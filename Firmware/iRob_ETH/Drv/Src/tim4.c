#include "tim4.h"

void vTim4_initEncoder(void){
	// Enable TIM4 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	TIM_EncoderInterfaceConfig(
		TIM4,
		TIM_EncoderMode_TI12,
		TIM_ICPolarity_Rising,
		TIM_ICPolarity_Rising
	);
	
	TIM_SetAutoreload(TIM4, 0xFFFF);
	TIM_Cmd(TIM4, ENABLE);
}
