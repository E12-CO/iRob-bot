#ifndef APP_MSGTYPE_H
#define APP_MSGTYPE_H

#include "stdint.h"

typedef struct __attribute__((packed)){
	
	uint8_t u8RbcHeader[2];

	union{
		uint8_t u8Cmd;
		struct{
			uint8_t bIndex	  :5;// Bit 0-4
			uint8_t bType     :2;// Bit 5-6
			uint8_t bRW       :1;// Bit 7
		}regBit;    
	};

	uint8_t u8DataLength;
	
}tRosClientCommand;

enum eCommandType{
	eCOMMAND_INVALID	= 0,
	eCOMMAND_PARAM		= 1,
	eCOMMAND_SETPOINT	= 2,
	eCOMMAND_DEBUG		= 3,
};

uint8_t u8AppMsg_init(uint8_t *u8BufPtr);

#endif