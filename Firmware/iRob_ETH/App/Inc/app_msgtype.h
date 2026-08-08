#ifndef APP_MSGTYPE_H
#define APP_MSGTYPE_H

#include "stdint.h"
#include "app_tcpServer.h"
#include "app_controlLoop.h"

#define MAX_DATA_LENGTH		252

typedef struct __attribute__((packed)){
	
	uint8_t u8RbcHeader[2];// "RB" header from Client, "JB" header from server

	union{
		uint8_t u8Cmd;
		struct{
			uint8_t bIndex	  :5;// Bit 0-4 for command index
			uint8_t bType     :2;// Bit 5-6 for command type
			uint8_t bRW       :1;// Bit 7 for command read/write
		}regBit;    
	};

	uint8_t u8DataLength;// Read / Write data length
	
	uint8_t u8InDataPtr[MAX_DATA_LENGTH];
}tRosClientCommand;

enum eCommandType{
	eCOMMAND_INVALID	= 0,
	eCOMMAND_PARAM		= 1,
	eCOMMAND_CONTROL	= 2,
	eCOMMAND_DEBUG		= 3,
};

enum eReadWriteType{
	eRW_READ			= 0,
	eRW_WRITE			= 1 
};

enum eParamIndex{
	ePARAM_CTRL_STAT		= 0,// Parameter Status
	ePARAM_CTRL_KP 			= 1,// Control Kp
	ePARAM_CTRL_KI 			= 2,// Control Ki
	ePARAM_CTRL_KD 			= 3,// Control Kd
	ePARAM_CTRL_KFF 		= 4,// Control Kfeedforward
	ePARAM_CTRL_MIN			= 5,// Control output min
	ePARAM_CTRL_MAX			= 6,// Control output max
	ePARAM_CTRL_KPIDFF		= 7,// Control Kp, Ki, Kd and Kff in one shot
	ePARAM_CTRL_MINMAX		= 8,// Control output min and max in one shot
	ePARAM_CTRL_ENC_CPR 	= 9,// Rotary encoder count per revolution
	ePARAM_CTRL_GEAR_RATIO	= 10,// Motor to output shaft gear ratio

	
	ePARAM_COMMIT			= 31,// Commit parameter and mark it ready to start control loop
};

typedef struct{
	union{
		uint8_t u8ParamStat;
		struct{
			uint8_t bKpConfigured 		:1;
			uint8_t bKiConfigured		:1;
			uint8_t bKdConfigured		:1;
			uint8_t	bKffConfigured		:1;
			
			uint8_t bCtrlMinConfigured	:1;
			uint8_t bCtrlMaxConfigured	:1;
			uint8_t bEncCprConfigured	:1;
			uint8_t bGearRatioConfigured:1;
		}regBit;
	};
}tParameterStatus;

enum eControlIndex{
	eCONTROL_CTRL_STAT		= 0,// Control loop status
	eCONTROL_CTRL_SETPOINT	= 1,// Control loop setpoint
	eCONTROL_CTRL_FEEDBACK	= 2,// Control loop feedback
	
	eCONTROL_CTRL_LOOP_RATE = 30,// Control loop rate
	eCONTROL_CTRL_ON_OFF	= 31,// Control loop On/Off	
};

enum eDebugIndex{
	eDEBUG_STAT				= 0,// Debug status
	eDEBUG_RJ45_VENDOR		= 1,// Debug command to select the RJ45 vendor
	eDEBUG_SWO_ON_OFF		= 2,// Debug command to enable/disable SWO
};

typedef struct{
	union{
		uint8_t u8DebugStat;
		struct{
			uint8_t bRJ45IsWurth 		:1;
			uint8_t bSWOEnabled			:1;
			uint8_t bReserved			:6;
		}regBit;
	};
}tDebugingStatus;

uint8_t u8AppMsg_init(void);
void vAppMsg_processInputData(void);
void vAppMsg_sendEncoderData(void);

#endif