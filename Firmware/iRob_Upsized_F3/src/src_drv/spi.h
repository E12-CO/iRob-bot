#ifndef SPI_H
#define SPI_H

#include <stm32f303xc.h>

void spi_init();
uint8_t spi_getTXEmpty();
uint8_t spi_getRXNotEmpty();
void spi_write(uint8_t data);
uint8_t spi_read();

void spi_writeThenReadRequest(
	uint8_t *write_Ptr,
	uint8_t *read_Ptr,
	uint8_t xfer_count,
	void (*spi_select),
	void (*spi_deselect)
	);
void spi_ioPoll();
uint8_t spi_getFSMBusy();

#endif