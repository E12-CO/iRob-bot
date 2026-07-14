// iRob ETH V1 firmware
// Coded by TinLethax RB26
#include "ch32f20x.h"

#include "systick.h"
#include "gpio.h"
#include "tim5.h"
#include "tim2.h"

#include "app_tcpServer.h"
#include "app_msgtype.h"

int main(void){
	__disable_irq();
	
	// Update Clock 
	SystemCoreClockUpdate();
	// Setup 1kHz Systick, also used for ethernet stack tick
	vSystick_init(
		SYSCLK_FREQ_120MHz_HSE,
		1000
	);
	// Initialize GPIO
	vGpio_initPins();
	// Initialize TIM2 control loop timer
	//vTim2_initLoopTimer();
	// Initialize IP stack with the IP read from the Rotary Switch
	vAppTcp_serverInit(vGpio_readSlaveConfigPins());
	// Initialize Command parser
	u8AppMsg_init(u8SocketRecvBuf);
	
	__enable_irq();
	
	vAppTcp_createRosListenSocket();
	
	while(1){
		// Run the IP task in the super loop
		WCHNET_MainTask();
		
		// Handle connection and socket interrupts
		if(WCHNET_QueryGlobalInt()){
            vAppTcp_handleIpInterrupt();
        }
	}
}

