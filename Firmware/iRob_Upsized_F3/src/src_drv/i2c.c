#include "i2c.h"

// Private pointers
uint8_t *i2c_write_ptr;
uint8_t *i2c_read_ptr;

// Private variables
uint8_t i2c_fsm 				= 0;
uint8_t i2c_slave_addr	= 0;
uint8_t i2c_write_count = 0;
uint16_t i2c_read_count = 0;
uint8_t i2c_nbyte				= 0;

void i2c_init(){	
	// Switch I2C clock to SYSCLK
	RCC->CFGR3 |=
		RCC_CFGR3_I2C1SW;
	// Enable I2C clock
	RCC->APB1ENR |=
		RCC_APB1ENR_I2C1EN;
	
	// Init I2C1
	I2C1->CR1 = 0;
	
	I2C1->OAR1 |= 
		I2C_OAR1_OA1EN ;
	
	// 128kHz I2C
	I2C1->TIMINGR |= 0xF0000F0F;
	
	// Enable I2C
	I2C1->CR1 |= 
		I2C_CR1_PE;
}

void i2c_writeRequest(
	uint8_t i2c_addr,
	uint8_t *wr_ptr,
	uint8_t wr_cnt,
	uint8_t *rd_ptr,
	uint16_t rd_cnt
	){
	if(
		(i2c_addr < 1) ||
		(i2c_addr > 127)
	)
		return;
	
	if(wr_ptr == 0)
		return;
	
	if(wr_cnt == 0)
		return;
	
	if(rd_cnt != 0)
		if(rd_ptr == 0)
			return;
	
	i2c_slave_addr 	= i2c_addr;
	i2c_write_ptr 	= wr_ptr;
	i2c_write_count	=	wr_cnt;
	i2c_read_ptr		= rd_ptr;
	i2c_read_count	= rd_cnt;
	i2c_fsm = 1;// Kick start I2C FSM
}

void i2c_ioPoll(){

	switch(i2c_fsm){
		case 0:
		{
		
		}
		break;
	
		case 1:
		{
			I2C1->CR2 =
				((i2c_slave_addr & 0x7F) << 1) 	|
				(i2c_write_count << I2C_CR2_NBYTES_Pos) |
				((i2c_read_count > 0) ? 
					0 : 
					(1 << I2C_CR2_AUTOEND_Pos)
				);
			
			I2C1->CR2 |= 
				(1 << I2C_CR2_START_Pos);		// Generates start bit
			
			i2c_fsm = 2;
		}
		break;
		
		case 2:// Wait slave to ack back
		{
			// Halt if we got nack (No address match)
			if((I2C1->ISR & I2C_ISR_NACKF)){
				i2c_fsm = 127;
			}
			
			if(I2C1->ISR & I2C_ISR_TXIS){
				i2c_fsm = 3;
			}
			
		}
		break;
		
		case 3:// Write byte to slave
		{
			I2C1->TXDR = *i2c_write_ptr++;
			i2c_write_count--;
			i2c_fsm = 4;
		}
		break;
		
		case 4:// Write next byte or continue read
		{
			if(i2c_write_count == 0){
				// Write finished, check whether we will continue to read
				// or we'll just end it here
				if(i2c_read_count > 0){
					// No more write, but plenty of read
					if(!(I2C1->ISR & (I2C_ISR_TC | I2C_ISR_TCR)))
						break;
					
					i2c_fsm = 5;
				}else{
					// No more write, no more read -> exit
					if(!(I2C1->ISR & I2C_ISR_STOPF))
						break;
					
					i2c_fsm = 0;
				}
			}else{
				i2c_fsm = 2;// Continue write if there're more byte
			}
			
		}
		break;
		
		case 5:// Read data
		{
			
			if(i2c_read_count > 255)
				i2c_nbyte = 0xFF;
			else
				i2c_nbyte = (uint8_t)i2c_read_count;
			
			I2C1->CR2 =
				((i2c_slave_addr & 0x7F) << 1) 	|
				(1 << I2C_CR2_RD_WRN_Pos)	|
				(i2c_nbyte << I2C_CR2_NBYTES_Pos) |
				((i2c_read_count > 255) ? 
					(1 << I2C_CR2_RELOAD_Pos) : 
					(1 << I2C_CR2_AUTOEND_Pos)
				);
			
			// resend start bit
			I2C1->CR2 |= 
				(1 << I2C_CR2_START_Pos);		
			
			i2c_fsm = 6;
		}
		break;
		
		case 6:// Read data
		{
			if(!(I2C1->ISR & I2C_ISR_RXNE))
				break;
			
			*(i2c_read_ptr++) = I2C1->RXDR;
			i2c_read_count--;
			i2c_nbyte--;
			i2c_fsm = 7;
		}
		break;
		
		case 7:
		{
			if(i2c_read_count == 0){
				i2c_fsm = 0;
				return ;
			}
			
			if(i2c_nbyte == 0){
				i2c_fsm = 5;
			}else{
				i2c_fsm = 6;
			}
		
		}
		break;
		
		case 127:// halt due to I2C Nack
			break;
	
	}
	
}

uint8_t i2c_getFSMBusy(){
	return (i2c_fsm == 0) ? 0 : 1;
}

uint8_t i2c_getSlaveNack(){
	if(i2c_fsm != 127)
		return 0;
	
	i2c_fsm = 0;
	return 1;
}