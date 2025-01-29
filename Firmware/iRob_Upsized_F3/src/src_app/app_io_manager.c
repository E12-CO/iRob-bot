#include "app_io_manager.h"

// Private pointers
adns_data_t 			*adns_dataPointer_t;
l3gd20_data_t			*gyro_dataPointer_t;
lsm303dlhc_data_t	*acmag_dataPointer_t;

// Private variables
uint8_t manager_fsm = 0;

uint8_t manager_spi_runner_fsm = 0;
uint8_t manager_i2c_runner_fsm = 0;

uint8_t manager_spi_readRequest = 0;
uint8_t manager_i2c_readRequest = 0;

void app_io_manager_init(
	adns_data_t 			*adns_data_ptr_t,
	l3gd20_data_t			*gyro_data_ptr_t,
	lsm303dlhc_data_t	*acmag_data_ptr_t
	){
	
	adns_dataPointer_t = adns_data_ptr_t;
	gyro_dataPointer_t = gyro_data_ptr_t;
	acmag_dataPointer_t	=	acmag_data_ptr_t;
		
	spi_init();	
	i2c_init();	
}

void app_io_manager_runner(){

	switch(manager_fsm){	
		case 0:// Initialize ADNS5090 mouse sensor
		{
			switch(adns5090_init(CPI_1750)){
				case 1:// Wait until initialized 
					break;
				
				case 0:// Initialized, go on to init next sensor
					manager_fsm = 1;
					break;
				
				case 255:// Init failed
					manager_fsm = 1;
					break;
			}
			
		}
		break;
		
		case 1:// Initialize L3DG20 gyro sensor
		{
			switch(l3gd20_init()){
				case 1:// Wait until initialized 
					break;
				
				case 0:// Initialized, go on to init next sensor
					manager_fsm = 2;
					break;
				
				case 255:// Init failed
					manager_fsm = 255;
					break;
			}
		}
		break;
		
		case 2:// Initialize LSM303DLHC Accel+Mag sensor
		{
			switch(lsm303dlhc_init()){
				case 1:// Wait until initialized
					break;
				
				case 0:// Initialized, go on to run sensor
					manager_fsm = 127;
					break;
				
				case 255:// Init failed
					manager_fsm = 255;
					break;
			}
		}
		break;
		
		case 127:// Run the sensor IO
		{
			app_io_manager_SPIRunner();
			app_io_manager_I2CRunner();
		}
		break;
		
		case 255:// Error handler
		{
			
		}
		break;
		
	}

	spi_ioPoll();
	i2c_ioPoll();
}


uint8_t app_io_manager_getSensorReady(){
	return (manager_fsm == 127) ? 1 : 0;
}

void app_io_manager_SPITriggerNext(){
	manager_spi_readRequest = 1;
}

void app_io_manager_SPIRunner(){

	switch(manager_spi_runner_fsm){
		case 0:// Read Mouse sensor
		{
			if(spi_getFSMBusy())
				break;
				
			adns5090_getOdometry(adns_dataPointer_t);
			
			manager_spi_runner_fsm = 1;
		}
		break;
	
		case 1:// Read Gyro sensor
		{
			if(spi_getFSMBusy())	
				break;
			
			l3gd20_getGyroData(gyro_dataPointer_t);
			
			manager_spi_runner_fsm = 2;
		}
		break;
		
		case 2:// wait for data complete and post-processing
		{
			if(spi_getFSMBusy())
				break;
			
			l3gd20_applyOffset(gyro_dataPointer_t);
			
			manager_spi_runner_fsm = 3;
		}
		break;
		
		case 3:// Data is ready to be read
		{
			if(manager_spi_readRequest == 0)
				break;
			
			manager_spi_readRequest = 0;
			manager_spi_runner_fsm = 0;
		}
		break;
		
	}
	
}

uint8_t app_io_manager_getSPISensorDataReady(){
	return (manager_spi_runner_fsm == 3) ? 1 : 0;
}

void app_io_manager_I2CTriggerNext(){
	manager_i2c_readRequest = 1;
}

void app_io_manager_I2CRunner(){

	switch(manager_i2c_runner_fsm){
		case 0:// Read accelerometer data
		{
			if(i2c_getFSMBusy())
				break;
			
			lsm303dlhc_getAccelData(acmag_dataPointer_t);
			
			manager_i2c_runner_fsm = 1;
		}
		break;
		
		case 1:// Read magnetometer data
		{
			if(i2c_getFSMBusy())
				break;
			
			acmag_dataPointer_t->acc_x 	=
				acmag_dataPointer_t->acc_x >> 4;
			acmag_dataPointer_t->acc_y 	=
				acmag_dataPointer_t->acc_y >> 4;
			acmag_dataPointer_t->acc_z 	=
				acmag_dataPointer_t->acc_z >> 4;
			
			lsm303dlhc_getMagData(acmag_dataPointer_t);
			
			manager_i2c_runner_fsm = 2;
		}
		break;
		
		case 2:// Wait for data complete and post-processing
		{
			if(i2c_getFSMBusy())
				break;
			
			lsm303dlhc_convertMagEndianness(acmag_dataPointer_t);
			
			manager_i2c_runner_fsm = 3;
		}
		break;
		
		case 3:// Data is ready to be read
		{
			if(manager_i2c_readRequest == 0)
				break;
			
			manager_i2c_readRequest = 0;
			manager_i2c_runner_fsm = 0;
		}
		break;
		
	}
}

uint8_t app_io_manager_getI2CSensorDataReady(){
	return (manager_i2c_runner_fsm == 3) ? 1 : 0;
}