#ifndef APP_MOTOR_CTRL_H
#define APP_MOTOR_CTRL_H

#include <stm32f303xc.h>

#include "tim_pwm.h"
#include "tim_qdec.h"

#include "app_gpio.h"

typedef struct{
	float m1_rpm;
	float m2_rpm;
	float m3_rpm;
	float m4_rpm;
}motor_rpm_t;

typedef struct{
	const float Kp;
	const float Ki;

	
	float error;
	
	float Intg_accmu_dt;
	
	float out;
}pid_t;

void app_motor_control_init(
	motor_rpm_t *motor_command_ptr_t, 
	motor_rpm_t	*motor_feedback_ptr_t
	);

void app_motor_control_doEncoder();
void app_motor_control_doPID();
void app_motor_control_commandMotor();

#endif