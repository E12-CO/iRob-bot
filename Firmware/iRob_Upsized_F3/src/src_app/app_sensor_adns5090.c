// ADNS5090 optical flow odometry sensor

#include "app_sensor_adns5090.h"

// Private variables
uint8_t adns_write_buffer[4] = {0};
uint8_t adns_read_buffer[2] = {0};

uint8_t adns_fsm = 0;

void adns5090_selectSPI(){
	SEL_ADNS;	
}

void adns5090_deselectSPI(){
	DESEL_ADNS;	
}

void adns5090_writeReg(
	uint8_t reg, 
	uint8_t value){
	
	adns_write_buffer[0] = reg | 0x80;
	adns_write_buffer[1] = value;
		
	spi_writeThenReadRequest(
		adns_write_buffer,
		adns_read_buffer,
		2,
		adns5090_selectSPI,
		adns5090_deselectSPI
	);
		
}

void adns5090_readReg(
	uint8_t reg
	){

	adns_write_buffer[0] = reg;
	adns_write_buffer[1] = 0x00;
		
	spi_writeThenReadRequest(
		adns_write_buffer,
		adns_read_buffer,
		2,
		adns5090_selectSPI,
		adns5090_deselectSPI
	);
		
}

uint8_t adns5090_init(uint8_t CPI){

	switch(adns_fsm){
		case 0:// Probe Device product ID
		{
			adns5090_readReg(
				REG_PROD_ID
			);
			
			adns_fsm = 1;// Then wait for data
		}
		break;
		
		case 1:// Wait for data and check it
		{
			if(spi_getFSMBusy())
				break;
			
			// check ID
			if(adns_read_buffer[1] == ADNS5090_ID)
				adns_fsm = 2;
			else{
				adns_fsm = 255;// Enter error case
				return 255;
			}
		}
		break;
	
		case 2:// Set CPI
		{
			adns5090_writeReg(
				REG_MOUSE_CTRL,
				(1 << 5)		|
				CPI
			);
			
			adns_fsm = 3;
		}
		break;
		
		case 3:// Confirm CPI write by enable mouse control
		{
			if(spi_getFSMBusy())
				break;
			
			adns5090_writeReg(
				REG_MOUSECTRL_EN,
				0x10
			);
			
			adns_fsm = 4;
		}
		break;
		
		case 4:// Initialized 
		{
			if(spi_getFSMBusy())
				break;
			
			adns_fsm = 0;
			return 0;
		}
		break;
		
		case 255:
		default:
			return 255;
	}

	return 1;
}

void adns5090_getOdometry(adns_data_t *adns_ptr_t){
	adns_ptr_t->padding = REG_MOTION_BURST;
	
	
	spi_writeThenReadRequest(
		(uint8_t *)adns_ptr_t,
		(uint8_t *)adns_ptr_t,
		4,
		adns5090_selectSPI,
		adns5090_deselectSPI
	);
	
}