#include "app_controlLoop.h"

volatile __attribute__((section("ctrl_var"))) tCtrlPIDVar 		tPIDSpeedCtrl;
volatile __attribute__((section("ctrl_var"))) tCtrlLimit		tLimitSpeedCtrl;
volatile __attribute__((section("ctrl_var"))) tEncoderVar		tEncoderParam;
volatile __attribute__((section("ctrl_var"))) tControlStatus	tCtrlLoopStatus;
volatile __attribute__((section("ctrl_var"))) tEncoderEstimator tEncoderFilter;


const uint8_t u8LoopTickTable[6] = {
	40,		// 50Hz  - 20ms - 40 ticks
	20,		// 100Hz - 10ms - 20 ticks
	8,		// 250Hz - 4ms - 8 ticks
	4,  	// 500Hz - 2ms - 4 ticks
	2,  	// 1kHz	 - 1ms - 2 ticks
	0, 		// 2kHz	 - 0.5ms - 0 ticks
};

#define MAIN_LOOP_DT	0.0005f	// 500us

uint8_t u8ControlLoopRateNow = eLOOP_RATE_50HZ;
volatile __attribute__((section("ctrl_var"))) uint32_t u32LoopTick;
	
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
	
	// Get the current encoder position
	tEncoderParam.u16CurrentEncCount = GET_ENC_CNT;

	// Prediction step, predict the encoder position by velocity
	tEncoderFilter.f32Position += tEncoderFilter.f32Velocity * MAIN_LOOP_DT;
	// roll over and roll under handling
	// Because our TIM encoder counter count from 0 to 63365 and back to 0
	if(tEncoderFilter.f32Position > 65536.0f)
		tEncoderFilter.f32Position -= 65536.0f;
	if(tEncoderFilter.f32Position < 0.0f)
		tEncoderFilter.f32Position += 65536.0f;
	
	// Error comparator, compare the predicted position with the actual position
	tEncoderFilter.f32PositionDiff = 
			tEncoderParam.u16CurrentEncCount - 
			tEncoderFilter.f32Position;
	
	// Handling the encoder diff roll over
	if(tEncoderFilter.f32PositionDiff > 32767.0f)// rotation backward from 0 back to 65535 
		tEncoderFilter.f32PositionDiff -= 65536.0f;
	if(tEncoderFilter.f32PositionDiff < -32768.0f)// rotation forward from 65535 to 0
		tEncoderFilter.f32PositionDiff += 65536.0f;
	
	// Update step, update the final encoder position value as well as velocity
	tEncoderFilter.f32Position += 
		tEncoderFilter.f32Kp			*
		tEncoderFilter.f32PositionDiff;
	tEncoderFilter.f32Velocity += 
		tEncoderFilter.f32Ki			*
		tEncoderFilter.f32PositionDiff 	* 
		MAIN_LOOP_DT;
	
	if(tPIDSpeedCtrl.u32ControlLoopCanRun > 0)
		tPIDSpeedCtrl.u32ControlLoopCanRun--;
	
	if(
		(tCtrlLoopStatus.regBit.bControlRunning == 1) 	&& 
		(tPIDSpeedCtrl.u32ControlLoopCanRun == 0)		
	){
		// Update the tick according to the loop rate
		tPIDSpeedCtrl.u32ControlLoopCanRun = u32LoopTick;
		
		// Calculate the error
		tPIDSpeedCtrl.f32Err = tPIDSpeedCtrl.f32Setpoint - tEncoderFilter.f32Velocity;
		
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
	}else{// If control loop is not running
		tPIDSpeedCtrl.f32Setpoint 	= 0.0f;
		tPIDSpeedCtrl.f32Intg		= 0.0f;
		tPIDSpeedCtrl.f32Diff		= 0.0f;
		
	}
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void vAppControl_init(void){
	vTim2_startLoopTimer();
	vAppControl_setControlRun(0);
	
	// Setup the initial value for the velocity estimator
	tEncoderFilter.f32Position = 0.0f;
	tEncoderFilter.f32Velocity = 0.0f;
	tEncoderFilter.f32PositionDiff = 0.0f;
	
	tEncoderFilter.f32Kp = 0.25f;
	tEncoderFilter.f32Ki = 2000.0f;
}

void vAppControl_setControlRun(uint8_t u8OnOff){
	tCtrlLoopStatus.regBit.bControlRunning = u8OnOff ? 1 : 0;
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
			u32LoopTick = u8LoopTickTable[u8ControlLoopRateNow];
		}
		
		default:
			return 1;
	}
	
	return 0;
}

uint8_t u8AppControl_getControlRate(void){
	return u8ControlLoopRateNow;
}
