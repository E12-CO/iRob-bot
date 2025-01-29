// iRob upsized project - STM32F303 DISCO as a Low level controller.
#include "app_robot_runner.h"

void NMI_Handler(void){
	while(1);
}

void HardFault_Handler(void){
	while(1);
}

void MemManage_Handler(void){
	while(1);
}

void BusFault_Handler(void){
	while(1);
}

void UsageFault_Handler(void){
	while(1);
}

int main(){

	app_robot_appInit();
	
	while(1){
		app_robot_runner();
	}

}
