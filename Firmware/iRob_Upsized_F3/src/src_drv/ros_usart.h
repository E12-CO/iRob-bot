#ifndef ROS_USART_H
#define ROS_USART_H

#include "stm32f303xc.h"

enum BAUDRATE{
	BAUD_UNKNOWN 	= 0,
	BAUD_9600			=	1,
	BAUD_115200		= 2,
	BAUD_230400		= 3
};

uint8_t ROSusart_init(
	uint8_t baud_enum,
	void *registerRxCallbackFunctPtr);

uint8_t ROSusart_checkTxEmpty();
uint8_t ROSusart_checkTxComplete();
void ROSusart_transmitData(uint8_t data);

#endif