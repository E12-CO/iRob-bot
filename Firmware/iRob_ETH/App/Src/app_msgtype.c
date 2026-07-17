#include "app_msgtype.h"

// Pointer used to easily parse command
tRosClientCommand *tClientCmdPtr;// Pointer for parsing command from ROS client
tRosClientCommand *tServerCmdPtr;// Pointer for assembling return command 

// Prototype functions
uint8_t u8AppMsg_handleParameter(
	uint8_t u8ParamIndex, 
	uint8_t u8ParamReadWrite,
	uint8_t *pParamDataPtr, 
	uint8_t u8ParamLength
	);

uint8_t u8AppMsg_handleControl(
	uint8_t u8ControlIndex, 
	uint8_t u8ControlReadWrite,
	uint8_t *pControlDataPtr, 
	uint8_t u8ControlLength
	);

// Private data
tParameterStatus 	tParamStatus;

uint8_t u8ProcessInputReturn = 0;
uint32_t u32TxLength = 0;

uint8_t u8AppMsg_init(void){
	tClientCmdPtr = (tRosClientCommand*)&u8SocketRecvBuf;
	if(tClientCmdPtr == 0)
		return 1;
	
	tServerCmdPtr = (tRosClientCommand*)&u8SocketSendBuf;
	if(tServerCmdPtr == 0)
		return 1;
	
	// Setup the command reply header
	tServerCmdPtr->u8RbcHeader[0] = 'J';
	tServerCmdPtr->u8RbcHeader[1] = 'B';
	
	return 0;
}

void vAppMsg_processInputData(void){
	// Make sure that we don't process null pointer
	if(tClientCmdPtr == 0)
		return;
	
	// Check header - return if no header match
	if(
		(tClientCmdPtr->u8RbcHeader[0] != 'R') ||
		(tClientCmdPtr->u8RbcHeader[1] != 'B')
	)
		return;
	
	// Check the over-length
	if(tClientCmdPtr->u8DataLength > MAX_DATA_LENGTH)
		return;
	
	// Reply to client with same command byte
	tServerCmdPtr->u8Cmd = tClientCmdPtr->u8Cmd;
	
	// Check command
	switch(tClientCmdPtr->regBit.bType){
		case eCOMMAND_PARAM:// Parameter type data
		{
			u8ProcessInputReturn = 
				u8AppMsg_handleParameter(
					tClientCmdPtr->regBit.bIndex,
					tClientCmdPtr->regBit.bRW,
					tClientCmdPtr->u8InDataPtr,
					tClientCmdPtr->u8DataLength
				);
		}
		break;
		
		case eCOMMAND_CONTROL:// Control type data
		{
			u8ProcessInputReturn = 
				u8AppMsg_handleControl(
					tClientCmdPtr->regBit.bIndex,
					tClientCmdPtr->regBit.bRW,
					tClientCmdPtr->u8InDataPtr,
					tClientCmdPtr->u8DataLength
				);
		}
		break;
		
		case eCOMMAND_DEBUG:// Debug type data
		{
		
		}
		break;
		
		case eCOMMAND_INVALID:
		default:
			return;
	
	}
	
	// Setup the transfer to client if it's the read command
	if(tServerCmdPtr->regBit.bRW == eRW_READ){
		u32TxLength = tServerCmdPtr->u8DataLength + 4; // Header + Cmd + Length + Data
		WCHNET_SocketSend(
			1, 
			(uint8_t *)&tServerCmdPtr,
			&u32TxLength
			);
	}
	
}

uint8_t u8AppMsg_handleParameter(
	uint8_t u8ParamIndex, 
	uint8_t u8ParamReadWrite,
	uint8_t *pParamDataPtr, 
	uint8_t u8ParamLength
	){

	switch(u8ParamIndex){
		case ePARAM_CTRL_STAT:// Status - Read Only
		{
			if(u8ParamReadWrite != eRW_READ)
				return 1;
			
			tServerCmdPtr->u8DataLength = sizeof(uint8_t);
			tServerCmdPtr->u8InDataPtr[0] = tParamStatus.u8ParamStat;
		}
		break;
		
		case ePARAM_CTRL_KP:// PID Kp - Read Write
		{
			if(u8ParamLength != sizeof(float))
				return 1;
			
			if(u8ParamReadWrite == eRW_WRITE){
				tPIDSpeedCtrl.f32Kp = *(float *)pParamDataPtr;
				tParamStatus.regBit.bKpConfigured = 1;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float);
				*(float *)tServerCmdPtr->u8InDataPtr = tPIDSpeedCtrl.f32Kp;
			}
			
		}
		break;
		
		case ePARAM_CTRL_KI:// PID Ki - Read Write
		{
			if(u8ParamLength != sizeof(float))
				return 1;
			
			if(u8ParamReadWrite == eRW_WRITE){
				tPIDSpeedCtrl.f32Ki = *(float *)pParamDataPtr;
				tParamStatus.regBit.bKiConfigured = 1;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float);
				*(float *)tServerCmdPtr->u8InDataPtr = tPIDSpeedCtrl.f32Ki;
			}
			
		}
		break;
		
		case ePARAM_CTRL_KD:// PID Kd - Read Write
		{
			if(u8ParamLength != sizeof(float))
				return 1;
			
			if(u8ParamReadWrite == eRW_WRITE){
				tPIDSpeedCtrl.f32Kd = *(float *)pParamDataPtr;
				tParamStatus.regBit.bKdConfigured = 1;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float);
				*(float *)tServerCmdPtr->u8InDataPtr = tPIDSpeedCtrl.f32Kd;
			}
			
		}
		break;
		
		case ePARAM_CTRL_KFF:// PID Kfeedforward - Read Write
		{
			if(u8ParamLength != sizeof(float))
				return 1;
			
			if(u8ParamReadWrite == eRW_WRITE){
				tPIDSpeedCtrl.f32Kff = *(float *)pParamDataPtr;
				tParamStatus.regBit.bKffConfigured = 1;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float);
				*(float *)tServerCmdPtr->u8InDataPtr = tPIDSpeedCtrl.f32Kff;
			}
			
		}
		break;
		
		case ePARAM_CTRL_MIN:// Control output min - Read Write
		{
			if(u8ParamLength != sizeof(float))
				return 1;
			
			if(u8ParamReadWrite == eRW_WRITE){
				tLimitSpeedCtrl.f32ControlMin = *(float *)pParamDataPtr;
				tParamStatus.regBit.bCtrlMinConfigured = 1;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float);
				*(float *)tServerCmdPtr->u8InDataPtr = tLimitSpeedCtrl.f32ControlMin;
			}
			
		}
		break;
		
		case ePARAM_CTRL_MAX:// Control output max - Read Write
		{
			if(u8ParamLength != sizeof(float))
				return 1;
			
			if(u8ParamReadWrite == eRW_WRITE){
				tLimitSpeedCtrl.f32ControlMax = *(float *)pParamDataPtr;
				tParamStatus.regBit.bCtrlMaxConfigured = 1;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float);
				*(float *)tServerCmdPtr->u8InDataPtr = tLimitSpeedCtrl.f32ControlMax;
			}
			
		}
		break;
		
		case ePARAM_CTRL_KPIDFF:// PID gains all at once - Read Write
		{
			if(u8ParamLength != sizeof(float)*4)
				return 1;
			
			if(u8ParamReadWrite == eRW_WRITE){
				tPIDSpeedCtrl.f32Kp  = *(float *)(pParamDataPtr + 0x00);
				tPIDSpeedCtrl.f32Ki  = *(float *)(pParamDataPtr + 0x04);
				tPIDSpeedCtrl.f32Kd  = *(float *)(pParamDataPtr + 0x08);
				tPIDSpeedCtrl.f32Kff = *(float *)(pParamDataPtr + 0x0C);
				tParamStatus.u8ParamStat |= 0x0F;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float) * 4;
				*(float *)(tServerCmdPtr->u8InDataPtr + 0x00) = 
					tPIDSpeedCtrl.f32Kp;
				*(float *)(tServerCmdPtr->u8InDataPtr + 0x04) = 
					tPIDSpeedCtrl.f32Ki;
				*(float *)(tServerCmdPtr->u8InDataPtr + 0x08) = 
					tPIDSpeedCtrl.f32Kd;
				*(float *)(tServerCmdPtr->u8InDataPtr + 0x0C) = 
					tPIDSpeedCtrl.f32Kff;
			}
			
		}
		break;
		
		case ePARAM_CTRL_MINMAX:// Control output min max all at once - Read Write
		{
			if(u8ParamLength != sizeof(float)*2)
				return 1;
			
			if(u8ParamReadWrite == eRW_WRITE){
				tLimitSpeedCtrl.f32ControlMin  = *(float *)(pParamDataPtr + 0x00);
				tLimitSpeedCtrl.f32ControlMax  = *(float *)(pParamDataPtr + 0x04);
				tParamStatus.u8ParamStat |= (3 << 4);
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float) * 2;
				*(float *)(tServerCmdPtr->u8InDataPtr + 0x00) = 
					tLimitSpeedCtrl.f32ControlMin;
				*(float *)(tServerCmdPtr->u8InDataPtr + 0x04) = 
					tLimitSpeedCtrl.f32ControlMax;
			}
			
		}
		break;
		
		case ePARAM_CTRL_ENC_CPR:// Rotary encoder CPR value - Read Write
		{
			if(u8ParamLength != sizeof(uint32_t))
				return 1;
				
			if(u8ParamReadWrite == eRW_WRITE){
				tEncoderParam.u32EncoderCPR = *(uint32_t *)pParamDataPtr;
				tParamStatus.regBit.bEncCprConfigured = 1;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(uint32_t);
				*(uint32_t *)tServerCmdPtr->u8InDataPtr = tEncoderParam.u32EncoderCPR;
			}
			
		}
		break;
		
		case ePARAM_CTRL_GEAR_RATIO:// Motor to output shaft gear ratio - Read Write
		{
			if(u8ParamLength != sizeof(float))
				return 1;
			
			if(u8ParamReadWrite == eRW_WRITE){
				tEncoderParam.f32GearRatio = *(float *)pParamDataPtr;
				tParamStatus.regBit.bGearRatioConfigured = 1;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float);
				*(float *)tServerCmdPtr->u8InDataPtr = tEncoderParam.f32GearRatio;
			}
			
		}
		break;
		
		default:
			return 1;
	}	

	// Update the Control loop configured status
	if(tParamStatus.u8ParamStat == 0xFF)
		tCtrlLoopStatus.regBit.bControlConfigured = 1;
	else
		tCtrlLoopStatus.regBit.bControlConfigured = 0;
	
	return 0;
}
	
uint8_t u8AppMsg_handleControl(
	uint8_t u8ControlIndex, 
	uint8_t u8ControlReadWrite,
	uint8_t *pControlDataPtr, 
	uint8_t u8ControlLength
	){

	switch(u8ControlIndex){
		case eCONTROL_CTRL_STAT:// Control loop status - Read Only
		{
			if(u8ControlReadWrite != eRW_READ)
				return 1;
			
			tServerCmdPtr->u8DataLength = sizeof(uint8_t);
			tServerCmdPtr->u8InDataPtr[0] = tCtrlLoopStatus.u8CtrlStat;
			
		}
		break;
		
		case eCONTROL_CTRL_SETPOINT:// Control loop setpoint - Read Write
		{
			if(u8ControlLength != sizeof(float))
				return 1;
			
			if(u8ControlReadWrite == eRW_WRITE){
				tPIDSpeedCtrl.f32Setpoint = *(float *)pControlDataPtr;
			}else{
				tServerCmdPtr->u8DataLength = sizeof(float);
				*(float *)tServerCmdPtr->u8InDataPtr = tPIDSpeedCtrl.f32Setpoint;
			}
			
		}
		break;
		
		case eCONTROL_CTRL_LOOP_RATE:// Control loop rate - Read Write
		{
			if(u8ControlLength != sizeof(uint8_t))
				return 1;
			
			if(u8ControlReadWrite == eRW_WRITE){
				u8AppControl_setControlRate(*pControlDataPtr);
			}else{
				tServerCmdPtr->u8DataLength = sizeof(uint8_t);
				tServerCmdPtr->u8InDataPtr[0] = u8AppControl_getControlRate();
			}
			
		}
		break;
		
		case eCONTROL_CTRL_ON_OFF:// Control loop On/Off - Read Write
		{
			if(u8ControlLength != sizeof(uint8_t))
				return 1;
			
			if(u8ControlReadWrite == eRW_WRITE){
				// Prevent from starting the control loop if not configured
				if(
					(*pControlDataPtr == 1)				&&
					(tParamStatus.u8ParamStat != 0xFF)
				)
					return 1;
				vAppControl_setControlRun(*pControlDataPtr);
			}else{
				tServerCmdPtr->u8DataLength = sizeof(uint8_t);
				tServerCmdPtr->u8InDataPtr[0] = u8AppControl_getControlRunStatus();
			}
			
		}
		break;
		
		default:
			return 1;
	}
	
	return 0;
}
