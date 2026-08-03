#include "gpio.h"

GPIO_InitTypeDef tGPIOAinit = {0};
GPIO_InitTypeDef tGPIOBinit = {0};
//GPIO_InitTypeDef tGPIOCinit = {0};
GPIO_InitTypeDef tGPIODinit = {0};

void vGpio_initPins(void){

	RCC_APB2PeriphClockCmd(
		RCC_APB2Periph_AFIO		|	
		RCC_APB2Periph_GPIOA 	|
		RCC_APB2Periph_GPIOB	|
		RCC_APB2Periph_GPIOC	|
		RCC_APB2Periph_GPIOD,
		ENABLE
	);
	
	// Initialize GPIO A
	tGPIOAinit.GPIO_Pin 	= 
			GPIO_PA1_VINSENSE_P | 
			GPIO_PA2_VINSENSE_N	;
	tGPIOAinit.GPIO_Mode 	= GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &tGPIOAinit);
	
	// Initialize GPIO B - Output
//	tGPIOBinit.GPIO_Pin		= GPIO_PB3_DRVEN_R;
//	tGPIOBinit.GPIO_Mode	= GPIO_Mode_Out_PP;
//	tGPIOBinit.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &tGPIOBinit);
	
	// Initialize GPIO B - Input pullup
	tGPIOBinit.GPIO_Pin		= 
			GPIO_PB12_SW_1 | 
			GPIO_PB13_SW_2 | 
			GPIO_PB14_SW_4 | 
			GPIO_PB15_SW_8;
	tGPIOBinit.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &tGPIOBinit);
	GPIO_Write(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
	
	// Initialize GPIO B - Alternate functions Output
	tGPIOBinit.GPIO_Pin		= 
		GPIO_PB4_TIM3_CH1	|
		GPIO_PB5_TIM3_CH2	;
	tGPIOBinit.GPIO_Mode	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &tGPIOBinit);
	
	// Initialize GPIO B - Alternate functions Input
	tGPIOBinit.GPIO_Pin		=
		GPIO_PB6_TIM4_ENC_B	|
		GPIO_PB7_TIM4_ENC_A	;
	tGPIOBinit.GPIO_Mode	= GPIO_Mode_IPD;//GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &tGPIOBinit);
	
	// Select GPIO B Alternate function with AFIO
	// Remap TIM3 CH1,CH2 to PB4 and PB5
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
	// No need to remap the TIM4 CH1 and CH2
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);
	
	// Initialize GPIO D - Output
	tGPIODinit.GPIO_Pin		= GPIO_PD2_DRVEN_L;
	tGPIODinit.GPIO_Mode	= GPIO_Mode_Out_PP;
	tGPIODinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &tGPIODinit);
}

uint8_t vGpio_readIPConfigPins(void){
	return 15 - (uint8_t)((GPIO_ReadInputData(GPIOB) >> 12) & 0x0F);
}
