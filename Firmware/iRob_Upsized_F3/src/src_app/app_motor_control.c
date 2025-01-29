#include "app_motor_control.h"

#define ENC_CONST 	937.5f // (1 rev / (16ppr * 4) counts) * (1000Hz) * (60s / 1min)

#define DEADBAND_SPEED	10.0f

// Private variables
int32_t m1_from_float = 0;
int32_t m2_from_float = 0;
int32_t m3_from_float = 0;
int32_t m4_from_float = 0;


// Private typedefs
motor_rpm_t 		*motor_velcmd_t;
motor_rpm_t			*motor_velread_t;

motor_duty_t		motor_pwm_t;

encoder_cnt_t		enc_cnt_t;
encoder_cnt_t		prev_enc_cnt_t;

pid_t	motor1_pid_t = {
	.Kp = 1.0f,
	.Ki = 0.1f,

	.error = 0.0f,
	.Intg_accmu_dt = 0.0f,
	.out = 0.0f
};

pid_t motor2_pid_t = {
	.Kp = 1.0f,
	.Ki = 0.1f,

	.error = 0.0f,
	.Intg_accmu_dt = 0.0f,
	.out = 0.0f
};

pid_t motor3_pid_t = {
	.Kp = 1.0f,
	.Ki = 0.1f,

	.error = 0.0f,
	.Intg_accmu_dt = 0.0f,
	.out = 0.0f
};

pid_t motor4_pid_t = {
	.Kp = 1.0f,
	.Ki = 0.1f,

	.error = 0.0f,
	.Intg_accmu_dt = 0.0f,
	.out = 0.0f
};

void app_motor_control_init(
	motor_rpm_t *motor_command_ptr_t, 
	motor_rpm_t	*motor_feedback_ptr_t
	){
	
	motor_velcmd_t 	= motor_command_ptr_t;
	motor_velread_t	=	motor_feedback_ptr_t;
		
	tim_qdec_init();
	tim_pwm_init();
}

void app_motor_control_doEncoder(){
	tim_qdec_getCount(&enc_cnt_t);// Read count from the CNT registers

	// Calculate the dt of each encoder
	// Enc = (currect_count - previous_count) * constant
	// Enc then accumulate over the time and divide by 4
	// (simple 4 taps FIR moving average filter)
	motor_velread_t->m1_rpm += 
		(float)(enc_cnt_t.enc1 - prev_enc_cnt_t.enc1) * ENC_CONST;
	motor_velread_t->m1_rpm = motor_velread_t->m1_rpm * 0.25f;
	
	motor_velread_t->m2_rpm += 
		(float)(enc_cnt_t.enc2 - prev_enc_cnt_t.enc2) * ENC_CONST;
	motor_velread_t->m2_rpm = motor_velread_t->m2_rpm * 0.25f; 
	
	motor_velread_t->m3_rpm += 
		(float)(enc_cnt_t.enc3 - prev_enc_cnt_t.enc3) * ENC_CONST;
	motor_velread_t->m3_rpm = motor_velread_t->m3_rpm * 0.25f;
	
	motor_velread_t->m4_rpm += 
		(float)(enc_cnt_t.enc4 - prev_enc_cnt_t.enc4) * ENC_CONST;
	motor_velread_t->m4_rpm = motor_velread_t->m4_rpm * 0.25f;
	
	// Z^-1 the encoder data
	prev_enc_cnt_t.enc1 = enc_cnt_t.enc1;
	prev_enc_cnt_t.enc2 = enc_cnt_t.enc2;
	prev_enc_cnt_t.enc3 = enc_cnt_t.enc3;
	prev_enc_cnt_t.enc4 = enc_cnt_t.enc4;
}

void app_motor_control_doPID(){
	// Motor 1
	motor1_pid_t.error = 
		motor_velcmd_t->m1_rpm -
		motor_velread_t->m1_rpm ;
	
	motor1_pid_t.Intg_accmu_dt +=
		motor1_pid_t.error *	
		motor1_pid_t.Ki;
	
	motor1_pid_t.out = 
		(motor1_pid_t.Kp * motor1_pid_t.error) +
		motor1_pid_t.Intg_accmu_dt;
	
	// Capping output 
	if(motor1_pid_t.out > 4095.0f)
		motor1_pid_t.out = 4095.0f;
	if(motor1_pid_t.out < -4095.0f)
		motor1_pid_t.out = -4095.0f;
	
	// Velocity deadband
	if(
		(motor1_pid_t.out > -DEADBAND_SPEED) &&
		(motor1_pid_t.out < DEADBAND_SPEED)
	){
		motor1_pid_t.out = 0.0f;
		motor1_pid_t.Intg_accmu_dt = 0.0f;// reset integrator
	}
	
	
	// Motor 2
	motor2_pid_t.error = 
		motor_velcmd_t->m2_rpm -
		motor_velread_t->m2_rpm ;
	
	motor2_pid_t.Intg_accmu_dt +=
		motor2_pid_t.error *	
		motor2_pid_t.Ki;
	
	motor2_pid_t.out = 
		(motor2_pid_t.Kp * motor2_pid_t.error) +
		motor2_pid_t.Intg_accmu_dt;
	
	// Capping output 
	if(motor2_pid_t.out > 4095.0f)
		motor2_pid_t.out = 4095.0f;
	if(motor2_pid_t.out < -4095.0f)
		motor2_pid_t.out = -4095.0f;
	
	// Velocity deadband
	if(
		(motor2_pid_t.out > -DEADBAND_SPEED) &&
		(motor2_pid_t.out < DEADBAND_SPEED)
	){
		motor2_pid_t.out = 0.0f;
		motor2_pid_t.Intg_accmu_dt = 0.0f;// reset integrator
	}
	
	
	// Motor 3
	motor3_pid_t.error = 
		motor_velcmd_t->m3_rpm -
		motor_velread_t->m3_rpm ;
	
	motor3_pid_t.Intg_accmu_dt +=
		motor3_pid_t.error *	
		motor3_pid_t.Ki;
	
	motor3_pid_t.out = 
		(motor3_pid_t.Kp * motor3_pid_t.error) +
		motor3_pid_t.Intg_accmu_dt;
	
	// Capping output 
	if(motor3_pid_t.out > 4095.0f)
		motor3_pid_t.out = 4095.0f;
	if(motor3_pid_t.out < -4095.0f)
		motor3_pid_t.out = -4095.0f;
	
	// Velocity deadband
	if(
		(motor3_pid_t.out > -DEADBAND_SPEED) &&
		(motor3_pid_t.out < DEADBAND_SPEED)
	){
		motor3_pid_t.out = 0.0f;
		motor3_pid_t.Intg_accmu_dt = 0.0f;// reset integrator
	}
	
	
	// Motor 4
	motor4_pid_t.error = 
		motor_velcmd_t->m4_rpm -
		motor_velread_t->m4_rpm ;
	
	motor4_pid_t.Intg_accmu_dt +=
		motor4_pid_t.error *	
		motor4_pid_t.Ki;
	
	motor4_pid_t.out = 
		(motor4_pid_t.Kp * motor4_pid_t.error) +
		motor4_pid_t.Intg_accmu_dt;
	
	// Capping output 
	if(motor4_pid_t.out > 4095.0f)
		motor4_pid_t.out = 4095.0f;
	if(motor4_pid_t.out < -4095.0f)
		motor4_pid_t.out = -4095.0f;
	
	// Velocity deadband
	if(
		(motor4_pid_t.out > -DEADBAND_SPEED) &&
		(motor4_pid_t.out < DEADBAND_SPEED)
	){
		motor4_pid_t.out = 0.0f;
		motor4_pid_t.Intg_accmu_dt = 0.0f;// reset integrator
	}
	
}

void app_motor_control_commandMotor(){
	
	m1_from_float = (int32_t)motor1_pid_t.out;
	m2_from_float = (int32_t)motor2_pid_t.out;
	m3_from_float = (int32_t)motor3_pid_t.out;
	m4_from_float = (int32_t)motor4_pid_t.out;
	
	if(m1_from_float > 0){
		motor_pwm_t.m1 = (uint16_t)(m1_from_float);
		M1_INB_DN;
	}else if(m1_from_float < 0){
		motor_pwm_t.m1 = (uint16_t)(4095 + m1_from_float);
		M1_INB_UP;
	}else{
		motor_pwm_t.m1 = 0;
		M1_INB_DN;
	}
	
	if(m2_from_float > 0){
		motor_pwm_t.m2 = (uint16_t)(m2_from_float);
		M2_INB_DN;
	}else if(m2_from_float < 0){
		motor_pwm_t.m2 = (uint16_t)(4095 + m2_from_float);
		M2_INB_UP;
	}else{
		motor_pwm_t.m2 = 0;
		M2_INB_DN;
	}
	
	if(m3_from_float > 0){
		motor_pwm_t.m3 = (uint16_t)(m3_from_float);
		M3_INB_DN;
	}else if(m3_from_float < 0){
		motor_pwm_t.m3 = (uint16_t)(4095 + m3_from_float);
		M3_INB_UP;
	}else{
		motor_pwm_t.m3 = 0;
		M3_INB_DN;
	}
	
	if(m4_from_float > 0){
		motor_pwm_t.m4 = (uint16_t)(m4_from_float);
		M4_INB_DN;
	}else if(m4_from_float < 0){
		motor_pwm_t.m4 = (uint16_t)(4095 + m4_from_float);
		M4_INB_UP;
	}else{
		motor_pwm_t.m4 = 0;
		M4_INB_DN;
	}
	
	tim_pwm_updateCompare(&motor_pwm_t);
}