#ifndef APP_IO_MAN_H
#define APP_IO_MAN_H

#include "spi.h"
#include "i2c.h"

#include "app_sensor_l3gd20.h"
#include "app_sensor_adns5090.h"
#include "app_sensor_lsm303dlhc.h"

void app_io_manager_init(
	adns_data_t 			*adns_data_ptr_t,
	l3gd20_data_t			*gyro_data_ptr_t,
	lsm303dlhc_data_t	*acmag_data_ptr_t
	);

void app_io_manager_runner();

uint8_t app_io_manager_getSensorReady();

void app_io_manager_SPITriggerNext();
void app_io_manager_SPIRunner();
uint8_t app_io_manager_getSPISensorDataReady();

void app_io_manager_I2CTriggerNext();
void app_io_manager_I2CRunner();
uint8_t app_io_manager_getI2CSensorDataReady();

#endif