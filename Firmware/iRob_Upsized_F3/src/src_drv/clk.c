// Clock initialization for STM32F303 DISCO board

#include "clk.h"

void clk_init(){
	// We will be using the 8MHz external clock from the ST-Link
	RCC->CR |= RCC_CR_HSEON | RCC_CR_HSEBYP;
	// Wait for HSE to stable
	while(!(RCC->CR & RCC_CR_HSERDY));
	
	// Set up PLL to get from 8MHz to 72Mhz
	RCC->CFGR |= 
		RCC_CFGR_PLLMUL9							|	// Multiply 8MHz by 9 to get 72Mhz
		RCC_CFGR_PLLSRC_HSE_PREDIV		;	// Select HSE as PLL clock source
	// Turn PLL on
	RCC->CR |= RCC_CR_PLLON;
	// Wait until PLL is ready before switch to it
	while(!(RCC->CR & RCC_CR_PLLRDY));
	
	// Set flash latency to accomodate for 72Mhz clock
	FLASH->ACR |= (2 << FLASH_ACR_LATENCY_Pos);
	
	// Switch System clock to PLL
	RCC->CFGR	|= RCC_CFGR_SW_PLL;		// Select PLL as system clock source
	// wait until sysclock switched to PLL
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL));

	// DONE
}