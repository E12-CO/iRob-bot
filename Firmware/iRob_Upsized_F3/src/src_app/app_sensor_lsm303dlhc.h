#ifndef SENSOR_LSM303DLHC_H
#define SENSOR_LSM303DLHC_H

#include "i2c.h"

#define LSM303_ACCEL_ID		0x19	// 7bit I2C address
#define LSM303_MAG_ID			0x1E	

// Accelerometer 
#define ACC_CTRL_1				0x20
#define ACC_CTRL_2				0x21
#define ACC_CTRL_3				0x22
#define ACC_CTRL_4				0x23
#define ACC_CTRL_5				0x24
#define ACC_CTRL_6				0x25
#define ACC_OUT_X_L				0x28	

// Magnetometer
#define MAG_CRA_REG				0x00
#define MAG_CRB_REG				0x01
#define MAG_MR_REG				0x02
#define MAG_OUT_X_H				0x03

typedef struct __attribute__((packed)){
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
}lsm303dlhc_data_t;

uint8_t lsm303dlhc_init();

void lsm303dlhc_getAccelData(lsm303dlhc_data_t *lsm_ptr_t);
void lsm303dlhc_getMagData(lsm303dlhc_data_t *lsm_ptr_t);
void lsm303dlhc_convertMagEndianness(lsm303dlhc_data_t *lsm_ptr_t);

#endif