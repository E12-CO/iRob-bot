#include "gpio.h"

GPIO_InitTypeDef tGPIOAinit = {0};
GPIO_InitTypeDef tGPIOBinit = {0};
GPIO_InitTypeDef tGPIOCinit = {0};
GPIO_InitTypeDef tGPIODinit = {0};

uint8_t u8EthLedState = 0;

void vGpio_initPins(void){

	RCC_APB2PeriphClockCmd(
		RCC_APB2Periph_AFIO		|	
		RCC_APB2Periph_GPIOA 	|
		RCC_APB2Periph_GPIOB	|
		RCC_APB2Periph_GPIOC	|
		RCC_APB2Periph_GPIOD,
		ENABLE
	);
	
	// Initialize GPIO A - Analog input
	tGPIOAinit.GPIO_Pin 	= 
			GPIO_PA1_VINSENSE_P | 
			GPIO_PA2_VINSENSE_N	;
	tGPIOAinit.GPIO_Mode 	= GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &tGPIOAinit);
	
	// Initialize GPIO A - Output
	tGPIOAinit.GPIO_Pin		=
			GPIO_PA15_LED1_A;
	tGPIOAinit.GPIO_Mode	= GPIO_Mode_Out_PP;
	tGPIOAinit.GPIO_Speed	= GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &tGPIOAinit);
	
	// Initialize GPIO B - Output
	tGPIOBinit.GPIO_Pin		= GPIO_PB5_DRV_POL;
	tGPIOBinit.GPIO_Mode	= GPIO_Mode_Out_PP;
	tGPIOBinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &tGPIOBinit);
	
	// Initialize GPIO B - Input pullup
	tGPIOBinit.GPIO_Pin		= 
			GPIO_PB1_SW_0  |
			GPIO_PB12_SW_1 | 
			GPIO_PB13_SW_2 | 
			GPIO_PB14_SW_4 | 
			GPIO_PB15_SW_8;
	tGPIOBinit.GPIO_Mode	= GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &tGPIOBinit);
	GPIO_Write(
		GPIOB, 
		GPIO_PB1_SW_0 	| 
		GPIO_PB12_SW_1 	| 
		GPIO_PB13_SW_2 	| 
		GPIO_PB14_SW_4 	| 
		GPIO_PB15_SW_8
	);
	
	// Initialize GPIO B - Alternate functions Output
	tGPIOBinit.GPIO_Pin		= 
		GPIO_PB4_TIM3_CH1	;
	tGPIOBinit.GPIO_Mode	= GPIO_Mode_AF_PP;
	tGPIOBinit.GPIO_Speed	= GPIO_Speed_50MHz;
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
	
	// Initialize GPIO C - Output
	tGPIOCinit.GPIO_Pin		=
		GPIO_PC10_ACTLED	|
		GPIO_PC11_LED2_A	|
		GPIO_PC12_LINKLED	;
	tGPIOCinit.GPIO_Mode	= GPIO_Mode_Out_PP;
	tGPIOCinit.GPIO_Speed	= GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &tGPIOCinit);
	
	// Initialize GPIO D - Output
	tGPIODinit.GPIO_Pin		= GPIO_PD2_DRVEN;
	tGPIODinit.GPIO_Mode	= GPIO_Mode_Out_PP;
	tGPIODinit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &tGPIODinit);
	
	DRVEN_OFF;// Off the driver by default
}

uint8_t vGpio_readIPConfigPins(void){
	return (15 - (uint8_t)((GPIO_ReadInputData(GPIOB) >> 12) & 0x0F)) | ((GPIO_ReadInputData(GPIOB) & (1 << 1)) ? 0x00 : 0x10);
}

void vGpio_setLinkLed(uint8_t mode){
	if(mode != 0)
		u8EthLedState &= ~(1 << 0);
	else
		u8EthLedState |= (1 << 0);
}

void vGpio_setActLed(uint8_t mode){
	if(mode != 0)
		u8EthLedState &= ~(1 << 1);
	else
		u8EthLedState |= (1 << 1);
}

void vGpio_ledToggleTask(void){
	// Toggle the Act LED
	if(u8EthLedState & 0x02){
		if(u8EthLedState & 0x20){
			u8EthLedState &= ~0x20;
			GPIO_SetBits(GPIOA, GPIO_PA15_LED1_A);
			GPIO_ResetBits(GPIOC, GPIO_PC10_ACTLED);
		}else{
			u8EthLedState |= 0x20;
			GPIO_ResetBits(GPIOA, GPIO_PA15_LED1_A);
			GPIO_SetBits(GPIOC, GPIO_PC10_ACTLED);
		}
	}else{
		GPIO_SetBits(GPIOA, GPIO_PA15_LED1_A);
		GPIO_SetBits(GPIOC, GPIO_PC10_ACTLED);
	}
	
	// Toggle the Link LED
	if(u8EthLedState & 0x01){
		if(u8EthLedState & 0x10){
			u8EthLedState &= ~0x10;
			GPIO_SetBits(GPIOC, GPIO_PC11_LED2_A);
			GPIO_ResetBits(GPIOC, GPIO_PC12_LINKLED);
		}else{
			u8EthLedState |= 0x10;
			GPIO_ResetBits(GPIOC, GPIO_PC11_LED2_A);
			GPIO_SetBits(GPIOC, GPIO_PC12_LINKLED);
		}
	}else{
		GPIO_SetBits(GPIOC, GPIO_PC11_LED2_A);
		GPIO_SetBits(GPIOC, GPIO_PC12_LINKLED);
	}
}
