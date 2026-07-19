#include "app_controlLoop.h"

volatile __attribute__((section("ctrl_var"))) tCtrlPIDVar 		tPIDSpeedCtrl;
volatile __attribute__((section("ctrl_var"))) tCtrlLimit		tLimitSpeedCtrl;
volatile __attribute__((section("ctrl_var"))) tEncoderVar		tEncoderParam;
volatile __attribute__((section("ctrl_var"))) tControlStatus	tCtrlLoopStatus;
volatile __attribute__((section("ctrl_var"))) tIIRFilter		tEncoderFilter;

const float f32LoopRateDtTable[6] = {
	0.02f,	// 50Hz  - 20ms
	0.01f,	// 100Hz - 10ms
	0.004,	// 250Hz - 4ms
	0.002,  // 500Hz - 2ms
	0.001,  // 1kHz	 - 1ms
	0.0005, // 2kHz	 - 0.5ms
};

uint8_t u8ControlLoopRateNow = eLOOP_RATE_50HZ;
volatile __attribute__((section("ctrl_var"))) float f32LoopRateDt;
	
void __attribute__((section("ctrl_isr"))) TIM2_IRQHandler(void){

	// Check the setpoint range
	if(tPIDSpeedCtrl.f32Setpoint < tLimitSpeedCtrl.f32ControlMin){
		tPIDSpeedCtrl.f32Setpoint = tLimitSpeedCtrl.f32ControlMin;
		tCtrlLoopStatus.regBit.bControlErrorSP = 1;
	}else if(tPIDSpeedCtrl.f32Setpoint > tLimitSpeedCtrl.f32ControlMax){
		tPIDSpeedCtrl.f32Setpoint = tLimitSpeedCtrl.f32ControlMax;
		tCtrlLoopStatus.regBit.bControlErrorSP = 1;
	}else{
		tCtrlLoopStatus.regBit.bControlErrorSP = 0;
	}
	
	// Calculate the speed
	tEncoderParam.u16CurrentEncCount = GET_ENC_CNT;
	tEncoderFilter.f32Input[0] = 
		((tEncoderParam.u16CurrentEncCount -
		 tEncoderParam.u16PrevEncCount) 
		* 60.0f) / 
		(f32LoopRateDt * tEncoderParam.u32EncoderCPR);
	tEncoderParam.u16CurrentEncCount = tEncoderParam.u16PrevEncCount;
	
	tEncoderFilter.f32Output[0] = 
		(tEncoderFilter.f32Input[0] * tEncoderFilter.f32CoeffA[0]) + // x[n]
		(tEncoderFilter.f32Input[1] * tEncoderFilter.f32CoeffA[1]) + // x[n-1]
		(tEncoderFilter.f32Input[2] * tEncoderFilter.f32CoeffA[2]) + // x[n-2]
		(tEncoderFilter.f32Output[1] * tEncoderFilter.f32CoeffB[0]) + // y[n-1]
		(tEncoderFilter.f32Output[2] * tEncoderFilter.f32CoeffB[1]) ; // y[n-2]
	
	tEncoderParam.f32EncoderRPM = tEncoderFilter.f32Output[0];
	
	// Z^-1
	tEncoderFilter.f32Input[2] = tEncoderFilter.f32Input[1];
	tEncoderFilter.f32Input[1] = tEncoderFilter.f32Input[0];
	tEncoderFilter.f32Output[2] = tEncoderFilter.f32Output[1];
	tEncoderFilter.f32Output[1] = tEncoderFilter.f32Output[0];
	
	// Calculate the error
	tPIDSpeedCtrl.f32Err = tPIDSpeedCtrl.f32Setpoint - tEncoderFilter.f32Output[0];
	
	// Integrate the error
	tPIDSpeedCtrl.f32Intg += tPIDSpeedCtrl.f32Err * tPIDSpeedCtrl.f32Ki;
	
	// Differentiate the error
	tPIDSpeedCtrl.f32Diff = 
		(tPIDSpeedCtrl.f32Err - tPIDSpeedCtrl.f32PrevErr) *
		tPIDSpeedCtrl.f32Kd;
	
	// Calculate the actuator control command
	// P term + I term + D term + Feedforward term
	tPIDSpeedCtrl.f32Command = 
		(tPIDSpeedCtrl.f32Err * tPIDSpeedCtrl.f32Kp) +
		tPIDSpeedCtrl.f32Intg +
		tPIDSpeedCtrl.f32Diff +
		(tPIDSpeedCtrl.f32Kff * tPIDSpeedCtrl.f32Setpoint);
	
	// Apply the control limit and dead band
	
	// Limit (Saturation)
	if(tPIDSpeedCtrl.f32Command > 4095){
		tPIDSpeedCtrl.f32Command = 4095;
		tCtrlLoopStatus.regBit.bControlSaturate = 1;
	}else if(tPIDSpeedCtrl.f32Command < -4095){
		tPIDSpeedCtrl.f32Command = -4095;
		tCtrlLoopStatus.regBit.bControlSaturate = 1;
	}else{
		tCtrlLoopStatus.regBit.bControlSaturate = 0;
	}
	
	// Deadband
	if(
		(tPIDSpeedCtrl.f32Command > -10) &&
		(tPIDSpeedCtrl.f32Command < 10)
	)
		tPIDSpeedCtrl.f32Command = 0;
	
	
	tPIDSpeedCtrl.f32PrevErr = tPIDSpeedCtrl.f32Err;
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void vAppControl_init(void){
	vTim2_setLoopRate(LOOP_RATE_50HZ);
	vAppControl_setControlRun(0);
	
	// Setup the IIR filter coefficients
	tEncoderFilter.f32CoeffA[0] = 0.1f;
	tEncoderFilter.f32CoeffA[1] = 0.0f;
	tEncoderFilter.f32CoeffA[2] = 0.0f;
	tEncoderFilter.f32CoeffB[0] = 0.5f;
	tEncoderFilter.f32CoeffB[1] = 0.0f;
}

void vAppControl_setControlRun(uint8_t u8OnOff){
	tCtrlLoopStatus.regBit.bControlRunning = u8OnOff ? 1 : 0;
	if(tCtrlLoopStatus.regBit.bControlRunning == 0){
		vTim2_stopLoopTimer();
	}else{
		vTim2_startLoopTimer();
	}
}

uint8_t u8AppControl_getControlRunStatus(void){
	return tCtrlLoopStatus.regBit.bControlRunning;
}

uint8_t u8AppControl_setControlRate(uint8_t u8LoopRate){
	
	// Don't allow control loop rate change when it is running
	if(tCtrlLoopStatus.regBit.bControlRunning != 0)
		return 1;
	
	switch(u8LoopRate){
		case eLOOP_RATE_50HZ:
		case eLOOP_RATE_100HZ:
		case eLOOP_RATE_250HZ:
		case eLOOP_RATE_500HZ:
		case eLOOP_RATE_1KHZ:
		case eLOOP_RATE_2KHZ:
		{
			u8ControlLoopRateNow = u8LoopRate;
			f32LoopRateDt = f32LoopRateDtTable[u8ControlLoopRateNow];
			vTim2_setLoopRate(
				(u8LoopRate == eLOOP_RATE_50HZ) 	? LOOP_RATE_50HZ :
				(u8LoopRate == eLOOP_RATE_100HZ) 	? LOOP_RATE_100HZ :
				(u8LoopRate == eLOOP_RATE_250HZ) 	? LOOP_RATE_250HZ :
				(u8LoopRate == eLOOP_RATE_500HZ) 	? LOOP_RATE_500HZ :
				(u8LoopRate == eLOOP_RATE_1KHZ) 	? LOOP_RATE_1KHZ :
				(u8LoopRate == eLOOP_RATE_2KHZ) 	? LOOP_RATE_2KHZ : LOOP_RATE_50HZ
			);
		}
		
		default:
			return 1;
	}
	
	return 0;
}

uint8_t u8AppControl_getControlRate(void){
	return u8ControlLoopRateNow;
}
