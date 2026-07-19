#ifndef APP_CONTROLLOOP_H
#define APP_CONTROLLOOP_H

#include "tim2.h"
#include "tim4.h"
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

typedef struct{
	uint32_t u32EncoderCPR;
	float	 f32GearRatio;
	
	uint16_t u16CurrentEncCount;
	uint16_t u16PrevEncCount;

	float f32EncoderRPM;
}tEncoderVar;

typedef struct{
	float f32Input[3];
	float f32Output[3];
	float f32CoeffA[3];
	float f32CoeffB[2];
}tIIRFilter;

typedef struct{
	union{
		uint8_t u8CtrlStat;
		struct{
			uint8_t bControlConfigured	:1;
			uint8_t bControlRunning		:1;
			uint8_t bControlSaturate	:1;
			uint8_t	bControlErrorSP		:1;
			
			uint8_t bControlErrorFB		:1;
			uint8_t bControlErrorOSC	:1;
			uint8_t bReserved1			:2;
		}regBit;
	};
}tControlStatus;

enum eLOOP_RATE{
	eLOOP_RATE_50HZ		= 0,
	eLOOP_RATE_100HZ	= 1,
	eLOOP_RATE_250HZ	= 2,
	eLOOP_RATE_500HZ	= 3,
	eLOOP_RATE_1KHZ		= 4,
	eLOOP_RATE_2KHZ		= 5,
};

extern volatile tCtrlPIDVar 	tPIDSpeedCtrl;
extern volatile tCtrlLimit		tLimitSpeedCtrl;
extern volatile tEncoderVar		tEncoderParam;
extern volatile tControlStatus	tCtrlLoopStatus;

void vAppControl_init(void);
void vAppControl_setControlRun(uint8_t u8OnOff);
uint8_t u8AppControl_getControlRunStatus(void);
uint8_t u8AppControl_setControlRate(uint8_t u8LoopRate);
uint8_t u8AppControl_getControlRate(void);
#endif