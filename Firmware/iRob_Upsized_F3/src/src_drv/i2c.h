#ifndef I2C_H
#define I2C_H

#include <stm32f303xc.h>

void i2c_init();
void i2c_writeRequest(
	uint8_t i2c_addr,
	uint8_t *wr_ptr,
	uint8_t wr_cnt,
	uint8_t *rd_ptr,
	uint16_t rd_cnt
	);

void i2c_ioPoll();
uint8_t i2c_getFSMBusy();
uint8_t i2c_getSlaveNack();

#endif