#ifndef SENSOR_L3GD20_H
#define SENSOR_L3GD20_H

#include "spi.h"

#include "app_gpio.h"

#define	L3GD20_ID				0xD4

#define REG_WHO_AM_I		0x0F
#define REG_CTRL_REG1		0x20
#define REG_CTRL_REG2		0x21
#define REG_CTRL_REG3		0x22
#define REG_CTRL_REG4		0x23
#define REG_CTRL_REG5		0x24
#define REG_REFERENCE		0x25
#define REG_OUT_TEMP		0x26
#define REG_STATUS_REG	0x27
#define REG_OUT_X_L			0x28
#define REG_OUT_X_H			0x29
#define REG_OUT_Y_L			0x2A
#define REG_OUT_Y_H			0x2B
#define REG_OUT_Z_L			0x2C
#define REG_OUT_Z_H			0x2D
#define REG_FIFO_CTRL		0x2E
#define REG_FIFO_SRC		0x2F


typedef struct __attribute__((packed)){
	uint8_t padding;
	
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	
}l3gd20_data_t;


uint8_t l3gd20_init();
uint8_t l3gd20_calibrationRoutine();
void l3gd20_getGyroData(l3gd20_data_t *gyro_ptr_t);
void l3gd20_applyOffset(l3gd20_data_t *gyro_ptr_t);

#endif