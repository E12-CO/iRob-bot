#ifndef APP_CONTROLLOOP_H
#define APP_CONTROLLOOP_H

#include "tim2.h"
#include "core_cm3.h" 

typedef struct{
	float f32Kp;
	float f32Ki;
	float f32Kd;
	float f32Kff;
	
	float f32Setpoint;
	
	float f32Err;
	float f32PrevErr;
	float f32Intg;
	float f32Diff;
	float f32Command;
}tCtrlPIDVar;

typedef struct{
	float f32ControlMin;
	float f32ControlMax;
}tCtrlLimit;


#endif