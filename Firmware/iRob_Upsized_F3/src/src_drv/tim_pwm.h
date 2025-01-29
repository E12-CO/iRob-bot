#ifndef TIM_PWM_H
#define TIM_PWM_H

#include <stm32f303xc.h>

typedef struct{
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;
	uint16_t m4;
}motor_duty_t;

void tim_pwm_init();

void tim_pwm_updateCompare(
	motor_duty_t *duty_ptr_t
	);

#endif