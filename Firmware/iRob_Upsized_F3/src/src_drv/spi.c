// L3GD20 driver for F3 Disco board
#include "spi.h"

// Private pointer
uint8_t *wr_ptr;	// point to tx data
uint8_t *rd_ptr;	// point to rx data

void (*spi_sel)();
void (*spi_desel)();

// Private variables
uint8_t spi_fsm = 0;

uint8_t spi_xfer_count = 0;

void spi_init(){
	// enable SPI clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	// Configure SPI
	SPI1->CR1 |=
		SPI_CR1_SSM						| // Use software managed SPI
		SPI_CR1_SSI						|
		(5 << SPI_CR1_BR_Pos)	|	// Divide 72MHz clock by 64 to get 1.125MHz clock
		SPI_CR1_CPHA					|	// SPI mode 3
		SPI_CR1_CPOL					|
		SPI_CR1_MSTR					;	// Use SPI as master
	
	SPI1->CR2 |=
		SPI_CR2_FRXTH					|	// RXNE is set when there's 8 bit data inside the rx register
		(7 << SPI_CR2_DS_Pos)	; // Set data size to 8-bit
		
	// Enable SPI1
	SPI1->CR1 |= SPI_CR1_SPE;	
}


uint8_t spi_getTXEmpty(){
	return (SPI1->SR & SPI_SR_TXE) ? 1 : 0;
}

uint8_t spi_getRXNotEmpty(){
	return (SPI1->SR & SPI_SR_RXNE) ? 1 : 0;
}

uint8_t spi_getBusy(){
	return (SPI1->SR & SPI_SR_BSY) ? 1 : 0;
}

void spi_write(uint8_t data){
	*(__IO uint8_t *)&SPI1->DR = data;
}

uint8_t spi_read(){
	return *(__IO uint8_t *)&SPI1->DR;
}

void spi_writeThenReadRequest(
	uint8_t *write_Ptr,
	uint8_t *read_Ptr,
	uint8_t xfer_count,
	void (*spi_select),
	void (*spi_deselect)
	){
	
	if(write_Ptr == 0)
		return;
	
	if(read_Ptr == 0)
		return;
	
	if(xfer_count == 0)
		return;
	
	if(spi_select == 0)
		return;
	
	if(spi_deselect == 0)
		return;
	
	if(spi_fsm != 0)
		return;
	
	spi_sel 	= spi_select;
	spi_desel = spi_deselect;
	
	wr_ptr = write_Ptr;
	rd_ptr = read_Ptr;
	spi_xfer_count 	= xfer_count;
	spi_fsm = 1;
}
	
void spi_ioPoll(){

	switch(spi_fsm){
		case 0:
		{	
			
		}
		break;
		
		case 1:// Select SPI
		{
			spi_sel();
			spi_fsm = 2;
		}
		break;
	
		case 2:// Write data to SPI data register
		{
			if(spi_getTXEmpty()){
				*((__IO uint8_t *)&SPI1->DR) = *wr_ptr;
				wr_ptr++;
				spi_fsm = 3;
			}
			
		}
		break;
		
		case 3:// Read data back from SPI data register
		{
			if(spi_getRXNotEmpty()){
				*rd_ptr = *(__IO uint8_t *)&SPI1->DR;
				rd_ptr++;
				spi_xfer_count--;
				spi_fsm = 4;
			}
		}
		break;
		
		case 4:// check transfer count
		{
			if(spi_xfer_count == 0){
				spi_fsm = 0;
				spi_desel();
			}else{
				spi_fsm = 2;
			}
		}
		break;
		
	}
	
}

uint8_t spi_getFSMBusy(){
	return (spi_fsm == 0) ? 0 : 1;
}