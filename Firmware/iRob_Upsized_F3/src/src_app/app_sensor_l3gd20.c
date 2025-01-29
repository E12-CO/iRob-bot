// L3GD20 Gyro sensor driver
#include "app_sensor_l3gd20.h"

// Private variables
uint8_t gyro_write_buffer[4] = {0};
uint8_t gyro_read_buffer[2] = {0};

uint8_t gyro_fsm = 0;

// For calibration 
uint16_t cal_counter = 0;
int32_t cal_accmu_x = 0;
int32_t cal_accmu_y = 0;
int32_t cal_accmu_z = 0;


// Private typedefs
l3gd20_data_t l3gd20_offsetCal_t;

void l3gd20_selectSPI(){
	SEL_L3GD20;	
}

void l3gd20_deselectSPI(){
	DESEL_L3GD20;	
}

void l3gd20_writeReg(
	uint8_t reg, 
	uint8_t value){
	
	gyro_write_buffer[0] = reg;
	gyro_write_buffer[1] = value;
		

	spi_writeThenReadRequest(
		gyro_write_buffer,
		gyro_read_buffer,
		2,
		l3gd20_selectSPI,
		l3gd20_deselectSPI
	);
	
}

void l3gd20_readReg(
	uint8_t reg
	){

	gyro_write_buffer[0] = reg | 0x80;
	gyro_write_buffer[1] = 0x00;
		
	spi_writeThenReadRequest(
		gyro_write_buffer,
		gyro_read_buffer,
		2,
		l3gd20_selectSPI,
		l3gd20_deselectSPI
	);
		
}


uint8_t l3gd20_init(){

	switch(gyro_fsm){
		case 0:// probe the sensor
		{
			
			l3gd20_readReg(
				REG_WHO_AM_I
			);
			gyro_fsm = 1;
		}
		break;
	
		case 1:// Check for the valid who am I code (0xD4)
		{
			if(spi_getFSMBusy())
				break;
			
			if(gyro_read_buffer[1] == L3GD20_ID)
				gyro_fsm = 2;
			else
				gyro_fsm = 255;
		}
		break;
		
		case 2:// Config Contrl register 1 -> Enable Gyro axis
		{

			l3gd20_writeReg(
				REG_CTRL_REG1,
				0x0F
			);
			gyro_fsm = 3;
		}
		break;
		
		case 3:// Config Contrl register 2 -> Config High pass filter
		{
			if(spi_getFSMBusy())
				break;
			
			l3gd20_writeReg(
				REG_CTRL_REG2,
				0x00
			);
			gyro_fsm = 4;
		}
		break;
		
		case 4:// Config Contrl register 3 -> Config interrupt out signal
		{
			if(spi_getFSMBusy())
				break;
			
			l3gd20_writeReg(
				REG_CTRL_REG3,
				0x00
			);
			gyro_fsm = 5;
		}
		
		case 5:// Config Contrl register 4 -> Config Data endianess and Gyro full scale
		{
			if(spi_getFSMBusy())
				break;
			
			l3gd20_writeReg(
				REG_CTRL_REG4,
				0x20						// +-2000 dps fullscale
			);
			gyro_fsm = 6;
		}
		
		case 6:// Config Contrl register 5 -> Config Highpass enabling, fifo and data routing
		{
			if(spi_getFSMBusy())
				break;
			
			l3gd20_writeReg(
				REG_CTRL_REG5,
				0x00
			);
			gyro_fsm = 7;
		}
		
		case 7:// Calibrate gyro sensor
		{
			if(l3gd20_calibrationRoutine())
				break;
			
			gyro_fsm = 8;
		}
		break;
		
		case 8:// Initialized
		{
			if(spi_getFSMBusy())
				break;
			
			gyro_fsm = 0;
			return 0;
		}
		break;
			
		case 255:
		{
			return 255;
		}
		break;
	}

	return 1;
}

uint8_t l3gd20_calibrationRoutine(){
	// acquire 2048 data sample
	if(cal_counter < 2049){
		if(!spi_getFSMBusy()){
			l3gd20_offsetCal_t.padding = REG_OUT_X_L | 0xC0;// Multiple byte read
			spi_writeThenReadRequest(
				(uint8_t *)&l3gd20_offsetCal_t,
				(uint8_t *)&l3gd20_offsetCal_t,
				7,
				l3gd20_selectSPI,
				l3gd20_deselectSPI
			);
		
			// Add value to acmmulator
			// At first sample will read zero on all axis
			cal_accmu_x += l3gd20_offsetCal_t.gyro_x;
			cal_accmu_y += l3gd20_offsetCal_t.gyro_y;
			cal_accmu_z += l3gd20_offsetCal_t.gyro_z;
			
			cal_counter++;
		}
	}else{
		cal_counter = 0;
		
		// Divide the total sum by 2048
		cal_accmu_x = cal_accmu_x >> 11;
		cal_accmu_y = cal_accmu_y >> 11;
		cal_accmu_z = cal_accmu_z >> 11;
		return 0;
	}
	
	
	return 1;
}

void l3gd20_getGyroData(l3gd20_data_t *gyro_ptr_t){
	gyro_ptr_t->padding = REG_OUT_X_L | 0xC0;// Multiple byte read
	spi_writeThenReadRequest(
		(uint8_t *)gyro_ptr_t,
		(uint8_t *)gyro_ptr_t,
		7,
		l3gd20_selectSPI,
		l3gd20_deselectSPI
	);

}

void l3gd20_applyOffset(l3gd20_data_t *gyro_ptr_t){
	gyro_ptr_t->gyro_x -= cal_accmu_x;
	gyro_ptr_t->gyro_y -= cal_accmu_y;
	gyro_ptr_t->gyro_z -= cal_accmu_z;
}