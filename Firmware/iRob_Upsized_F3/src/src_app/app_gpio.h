#ifndef APP_GPIO_H
#define APP_GPIO_H

// I2C sensor pins
#define I2C_SENSOR				GPIOB
#define I2C_SCL_Pin				6
#define I2C_SDA_Pin				7

// SPI sensor pins
#define SPI_SENSOR				GPIOA

#define SPI_SENSOR_SCK		5
#define SPI_SENSOR_MISO		6
#define SPI_SENSOR_MOSI		7

// SPI sensor select pin
#define SPI_L3DG20				GPIOE
#define SPI_CS_L3GD20			3	

#define DESEL_L3GD20				SPI_L3DG20->ODR |= (1 << SPI_CS_L3GD20)
#define SEL_L3GD20			SPI_L3DG20->ODR &= ~(1 << SPI_CS_L3GD20)

//
#define SPI_ADNS5090			GPIOA
#define SPI_CS_ADNS5090		0

#define DESEL_ADNS					SPI_ADNS5090->ODR |= (1 << SPI_CS_ADNS5090)
#define SEL_ADNS				SPI_ADNS5090->ODR &= ~(1 << SPI_CS_ADNS5090)

// ROS USART Pin
#define RUART_PORT				GPIOC
#define RUART_TX					4
#define RUART_RX					5

// Encoder Pin
#define ENC1_PORT					GPIOB
#define ENC1_1_TIM3				4
#define ENC1_2_TIM3				5

#define ENC2_PORT					GPIOD
#define ENC2_1_TIM2				3
#define ENC2_2_TIM2				4

#define ENC3_PORT					GPIOC
#define ENC3_1_TIM8				6
#define ENC3_2_TIM8				7

#define ENC4_PORT					GPIOD
#define ENC4_1_TIM4				12
#define ENC4_2_TIM4				13

// Motor driver pins
#define PWM_PORT					GPIOE
#define M1_PWM						9
#define M2_PWm						11
#define M3_PWM						13
#define	M4_PWM						14

#define M1_INB						9	// PB9
#define M1_INB_UP					GPIOB->ODR |= (1 << M1_INB)
#define M1_INB_DN					GPIOB->ODR &= ~(1 << M1_INB)

#define M2_INB						6	// PD6
#define M2_INB_UP					GPIOD->ODR |= (1 << M2_INB)
#define M2_INB_DN					GPIOD->ODR &= ~(1 << M2_INB)

#define M3_INB						12// PE12
#define M3_INB_UP					GPIOE->ODR |= (1 << M3_INB)
#define M3_INB_DN					GPIOE->ODR &= ~(1 << M3_INB)

#define M4_INB						15// PD15
#define M4_INB_UP					GPIOD->ODR |= (1 << M4_INB)
#define M4_INB_DN					GPIOD->ODR &= ~(1 << M4_INB)

#endif