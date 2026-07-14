#include "app_msgtype.h"

// Pointer used to point at the TCP buffer
uint8_t *u8TcpBufferPtr;

// Pointer used to easily parse command
tRosClientCommand *tClientCmdPtr;

uint8_t u8AppMsg_init(uint8_t *u8BufPtr){
	if(u8BufPtr == 0)
		return 1;

	u8TcpBufferPtr = u8BufPtr;
		
	return 0;
}
