#ifndef SENSOR_ADNS5090_H
#define SENSOR_ADNS5090_H

#include "spi.h"

#include "app_gpio.h"

#define ADNS5090_ID				0x29

#define REG_PROD_ID				0x00
#define REG_MOUSE_CTRL		0x0D
#define REG_MOUSECTRL_EN	0x21
#define REG_RESTMODE_CFG	0x45
#define REG_MOTION_BURST	0x63

#define CPI_1000					(0 << 2)
#define CPI_250						(1 << 2)
#define CPI_500						(2 << 2)
#define CPI_1250					(3 << 2)
#define CPI_1500					(4 << 2)
#define CPI_1750					(5 << 2)

typedef struct __attribute__((packed)){
	uint8_t padding;
	int8_t 	deltaX;
	int8_t 	deltaY;
	uint8_t squal;
}adns_data_t;

uint8_t adns5090_init(uint8_t CPI);
void adns5090_getOdometry(
	adns_data_t *adns_ptr_t);

#endif