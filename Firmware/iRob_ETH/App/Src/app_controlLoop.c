#include "app_controlLoop.h"

volatile tCtrlPIDVar 	tPIDSpeedCtrl;
volatile tCtrlLimit		tLimitSpeedCtrl;

//void TIM2_IRQHandler(void){

//	// Calculate the error
//	tPIDSpeedCtrl.f32Err = tPIDSpeedCtrl.f32Setpoint;
//	
//	// Integrate the error
//	tPIDSpeedCtrl.f32Intg += tPIDSpeedCtrl.f32Err * tPIDSpeedCtrl.f32Ki;
//	
//	// Differentiate the error
//	tPIDSpeedCtrl.f32Diff = 
//		(tPIDSpeedCtrl.f32Err - tPIDSpeedCtrl.f32PrevErr) *
//		tPIDSpeedCtrl.f32Kd;
//	
//	// Calculate the actuator control command
//	// P term + I term + D term + Feedforward term
//	tPIDSpeedCtrl.f32Command = 
//		(tPIDSpeedCtrl.f32Err * tPIDSpeedCtrl.f32Kp) +
//		tPIDSpeedCtrl.f32Intg +
//		tPIDSpeedCtrl.f32Diff +
//		(tPIDSpeedCtrl.f32Kff * tPIDSpeedCtrl.f32Setpoint);
//	
//	// Apply the control limit and dead band
//	
//	// Limit (Saturation)
//	if(tPIDSpeedCtrl.f32Command > tLimitSpeedCtrl.f32ControlMax){
//		tPIDSpeedCtrl.f32Command = tLimitSpeedCtrl.f32ControlMax;
//	}else if(tPIDSpeedCtrl.f32Command < -tLimitSpeedCtrl.f32ControlMax){
//		tPIDSpeedCtrl.f32Command = -tLimitSpeedCtrl.f32ControlMax;
//	}
//	
//	// Deadband
//	if(
//		(tPIDSpeedCtrl.f32Command > -tLimitSpeedCtrl.f32ControlMin) &&
//		(tPIDSpeedCtrl.f32Command < tLimitSpeedCtrl.f32ControlMin)
//	)
//		tPIDSpeedCtrl.f32Command = 0;
//	
//	
//	
//	tPIDSpeedCtrl.f32PrevErr = tPIDSpeedCtrl.f32Err;
//	
//	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//}

void vAppControl_init(void){



}
