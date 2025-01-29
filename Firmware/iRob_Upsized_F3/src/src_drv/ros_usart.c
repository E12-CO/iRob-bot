// USART 1 exclusively used to communicate with ROS on PC via ST-Link Virtual com port
#include "ros_usart.h"

// Private pointer
void (*readDataCallback)(uint8_t rx_data);


// USART1 callback handler
// call to the ros communication callback
void USART1_IRQHandler(void){
	
	// Add data to buffer when read is available
	if(USART1->ISR & USART_ISR_RXNE){
		readDataCallback((uint8_t)USART1->RDR);
		USART1->RQR |= USART_RQR_RXFRQ;
	}
	
}


uint8_t ROSusart_init(
	uint8_t baud_enum,
	void *registerRxCallbackFunctPtr){
	
	// Null pointer checking
	if(registerRxCallbackFunctPtr == 0)
		return 1;
	// Register the callback function if not null
	readDataCallback = registerRxCallbackFunctPtr;
		
	// Enable USART1 clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// Start basic configuration
	// UART 8N1 
	// RX interrupt and buffering
	USART1->CR1 |= 
		USART_CR1_RXNEIE	|	// Enable USART RX interrupt
		USART_CR1_TE			| // Enable USART Transmitter
		USART_CR1_RE 			;	// Enable USART Receiver
	
	// Set Baud rate (USART clock is 72MHz)
	// Value taken from DM00043574 manual
	switch(baud_enum){
		case BAUD_9600:
			USART1->BRR = 0x1D4C;
		break;
		
		case BAUD_115200:
			USART1->BRR = 0x0271;
		break;
		
		case BAUD_230400:
			USART1->BRR = 0x0139;
		break;
		
		default:
			USART1->BRR = 0x0271;
		break;
	
	}
	
	// Setup interrupt (NVIC)
	NVIC_SetPriority(USART1_IRQn, 3);
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	
	// Finally, enable USART1
	USART1->CR1 |= USART_CR1_UE;
	
	return 0;
}
	
uint8_t ROSusart_checkTxEmpty(){
	return (USART1->ISR & USART_ISR_TXE) ? 1 : 0;
}

uint8_t ROSusart_checkTxComplete(){
	return (USART1->ISR & USART_ISR_TC) ? 1 : 0;
}

void ROSusart_transmitData(uint8_t data){
	USART1->TDR = data;
}