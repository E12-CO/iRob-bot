#ifndef GPIO_H
#define GPIO_H

#include "ch32f20x.h"
#include "ch32f20x_rcc.h"
#include "ch32f20x_gpio.h"

void vGpio_initPins(void);


// PORT-A

#define GPIO_PA1_VINSENSE_P		GPIO_Pin_1
#define GPIO_PA2_VINSENSE_N		GPIO_Pin_2

// PORT-B
#define GPIO_PB4_TIM3_CH1		GPIO_Pin_4
#define GPIO_PB5_DRV_POL		GPIO_Pin_5
#define GPIO_PB6_TIM4_ENC_B		GPIO_Pin_6
#define GPIO_PB7_TIM4_ENC_A		GPIO_Pin_7
#define GPIO_PB12_SW_1			GPIO_Pin_12
#define GPIO_PB13_SW_2			GPIO_Pin_13
#define GPIO_PB14_SW_4			GPIO_Pin_14
#define GPIO_PB15_SW_8			GPIO_Pin_15

// PORT-C 
#define GPIO_PC6_RD_PHY_P		GPIO_Pin_6
#define GPIO_PC7_RD_PHY_N		GPIO_Pin_7
#define GPIO_PC8_TD_PHY_P		GPIO_Pin_8
#define GPIO_PC9_TD_PHY_N		GPIO_Pin_9
#define GPIO_PC11_ACTLED		GPIO_Pin_11
#define GPIO_PC12_LINKLED		GPIO_Pin_12

// PORT-D
#define GPIO_PD2_DRVEN			GPIO_Pin_2

#define DRVEN_ON				GPIO_WriteBit(GPIOD, GPIO_PD2_DRVEN, Bit_SET)
#define DRVEN_OFF				GPIO_WriteBit(GPIOD, GPIO_PD2_DRVEN, Bit_RESET)
	
#define DRV_H					GPIO_WriteBit(GPIOB, GPIO_PB5_DRV_POL, Bit_SET)
#define DRV_L					GPIO_WriteBit(GPIOB, GPIO_PB5_DRV_POL, Bit_RESET)

void vGpio_initPins(void);
uint8_t vGpio_readIPConfigPins(void);

#endif