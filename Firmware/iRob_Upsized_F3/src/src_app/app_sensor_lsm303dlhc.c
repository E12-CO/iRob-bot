// lsm303dlhc Accel and Mag sensor

#include "app_sensor_lsm303dlhc.h"

// Private pointers

// Private variables
uint8_t acmag_write_buffer[4] = {0};
uint8_t acmag_read_buffer[2] = {0};

uint8_t acmag_fsm = 0;

void lsm303dlhc_writeReg(
	uint8_t dev_addr,
	uint8_t reg,
	uint8_t data
	){

	acmag_write_buffer[0] = reg;
	acmag_write_buffer[1] = data;
		
	i2c_writeRequest(
		dev_addr,
		acmag_write_buffer,
		2,
		0,
		0
	);

}
	
void lsm303dlhc_readreg(
		uint8_t dev_addr,
		uint8_t reg
	){
	
	acmag_write_buffer[0] = reg;
		
	i2c_writeRequest(
		dev_addr,
		acmag_write_buffer,
		1,
		acmag_read_buffer,
		1
	);

}

uint8_t lsm303dlhc_init(){

	switch(acmag_fsm){
		case 0:// Probe the Accel sensor
		{
			lsm303dlhc_readreg(
				LSM303_ACCEL_ID,
				0x0F
			);
			
			acmag_fsm = 1;
		}
		break;
	
		case 1:// Check nack
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			acmag_fsm = 2;
		}
		break;
	
		case 2:// Probe the Mag sensor
		{
			lsm303dlhc_readreg(
				LSM303_MAG_ID,
				MAG_OUT_X_H
			);
			
			acmag_fsm = 3;
		}
		break;
		
		case 3:// Check nack
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			acmag_fsm = 4;
		}
		break;
		
		case 4:// Config Accel CTRL_1 -> 100Hz data rate
		{
			lsm303dlhc_writeReg(
				LSM303_ACCEL_ID,
				ACC_CTRL_1,
				0x57
			);
			
			acmag_fsm = 5;
		}
		break;
		
		case 5:// Config Accel CTRL_2
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			lsm303dlhc_writeReg(
				LSM303_ACCEL_ID,
				ACC_CTRL_2,
				0x00
			);
			
			acmag_fsm = 6;
		}
		break;
		
		case 6:// Config Accel CTRL_3
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			lsm303dlhc_writeReg(
				LSM303_ACCEL_ID,
				ACC_CTRL_3,
				0x00
			);
			
			acmag_fsm = 7;
		}
		break;
		
		case 7:// Config Accel CTRL_4 -> +-2g full range sensitivity, High resolution mode
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			lsm303dlhc_writeReg(
				LSM303_ACCEL_ID,
				ACC_CTRL_4,
				0x88
			);
			
			acmag_fsm = 8;
		}
		break;
		
		case 8:// Config Accel CTRL_5
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			lsm303dlhc_writeReg(
				LSM303_ACCEL_ID,
				ACC_CTRL_5,
				0x00
			);
			
			acmag_fsm = 9;
		}
		break;
		
		case 9:// Config Accel CTRL_6
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			lsm303dlhc_writeReg(
				LSM303_ACCEL_ID,
				ACC_CTRL_6,
				0x00
			);
			
			acmag_fsm = 10;
		}
		break;
		
		case 10:// Config mag CRA -> 75Hz
		{
		
			lsm303dlhc_writeReg(
				LSM303_MAG_ID,
				MAG_CRA_REG,
				0x1C
			);
			
			acmag_fsm = 11;
		}
		break;
		
		case 11:// Config mag CRB -> +-1.9 gauss full range sensitivity
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			lsm303dlhc_writeReg(
				LSM303_MAG_ID,
				MAG_CRB_REG,
				0xE0
			);
			
			acmag_fsm = 12;
		}
		break;
		
		case 12:// Config mag MR -> continuous conversion mode
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			lsm303dlhc_writeReg(
				LSM303_MAG_ID,
				MAG_MR_REG,
				0x00
			);
			
			acmag_fsm = 13;
		}
		break;
		
		case 13:// Done
		{
			if(i2c_getFSMBusy())
				return 1;
			
			if(i2c_getSlaveNack())
				return 255;
			
			acmag_fsm = 0;
			return 0;
		}
		break;
		
		case 255:
			return 255;
		break;
	}
	
	return 1;
}

void lsm303dlhc_getAccelData(lsm303dlhc_data_t *lsm_ptr_t){

	acmag_write_buffer[0] = ACC_OUT_X_L | 0x80;// Added 0x80 to get the multibyte read
		
	i2c_writeRequest(
		LSM303_ACCEL_ID,
		acmag_write_buffer,
		1,
		(uint8_t *)&lsm_ptr_t->acc_x,
		6
	);
	
}

void lsm303dlhc_getMagData(lsm303dlhc_data_t *lsm_ptr_t){

	acmag_write_buffer[0] = MAG_OUT_X_H;
		
	i2c_writeRequest(
		LSM303_MAG_ID,
		acmag_write_buffer,
		1,
		(uint8_t *)&lsm_ptr_t->mag_x,
		6
	);
	
}

void lsm303dlhc_convertMagEndianness(lsm303dlhc_data_t *lsm_ptr_t){
	
	lsm_ptr_t->mag_x = __REV16(lsm_ptr_t->mag_x);
	lsm_ptr_t->mag_y = __REV16(lsm_ptr_t->mag_y);
	lsm_ptr_t->mag_z = __REV16(lsm_ptr_t->mag_z);
	
}