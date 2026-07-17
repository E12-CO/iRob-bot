// iRob ETH V1 firmware
// Coded by TinLethax RB26
#include "ch32f20x.h"

#include "systick.h"
#include "gpio.h"
#include "tim2.h"
#include "tim4.h"

#include "app_tcpServer.h"
#include "app_msgtype.h"
#include "app_controlLoop.h"

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
	vTim2_initLoopTimer();
	// Initialize TIM4 encoder counter timer 
	vTim4_initEncoder();
	// Initialize control loop
	vAppControl_init();
	// Initialize IP stack with the IP read from the Rotary Switch
	vAppTcp_serverInit(vGpio_readIPConfigPins());
	// Initialize Command parser
	u8AppMsg_init();
	
	// Setup PB3 as SWO trace pin
	
	DBGMCU->CFGR |= 
		(1 << 5) | 		// TRACE_IOEN
		(0 << 6) ;		// TRACE_MODE -> Async UART type
	
	__enable_irq();
	
	vAppTcp_createRosListenSocket();
	
	while(1){
		// Run the IP task in the super loop
		WCHNET_MainTask();
		
		// Handle connection and socket interrupts
		if(WCHNET_QueryGlobalInt()){
            vAppTcp_handleIpInterrupt();
        }
		
		// Process input data
		if(
			u8AppTcp_isRosClientConnected() &&
			u8AppTcp_isThereDataToRead()
		){
			vAppMsg_processInputData();
		}

	}
}

