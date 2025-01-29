// Robot tasks runner
#include "app_robot_runner.h"

// Private variables
uint32_t controlRunner_millis = 0;
uint32_t sensorRunner_millis = 0;

// Private typedefs

// Sensor data
adns_data_t 			odom_data_t;			// ADNS-5090 mouse odometry
l3gd20_data_t			imu_gyro_data_t;	// L3GD20 Gyro data
lsm303dlhc_data_t	imu_acmag_data_t;	// Accel and Mag data

// Motor data
motor_rpm_t 		motor_cmdvel_t; // Command velocity for motor
motor_rpm_t			motor_fbvel_t;	// Encoder feedback	

void app_robot_gpioInit(){
	// Enable GPIO clock
	RCC->AHBENR |=
		RCC_AHBENR_GPIOAEN	|
		RCC_AHBENR_GPIOBEN	|
		RCC_AHBENR_GPIOCEN	|
		RCC_AHBENR_GPIODEN	|
		RCC_AHBENR_GPIOEEN	|
		RCC_AHBENR_GPIOFEN	;
	
	// I2C Sensor pins
	I2C_SENSOR->MODER |=
		(2 << (I2C_SCL_Pin * 2))	|
		(2 << (I2C_SDA_Pin * 2))	;
	
	I2C_SENSOR->AFR[0] |=
		(4 << (I2C_SCL_Pin * 4))	|
		(4 << (I2C_SDA_Pin * 4))	;
	
	I2C_SENSOR->OSPEEDR |= 
		(2 << (I2C_SCL_Pin * 2))	|
		(2 << (I2C_SDA_Pin * 2))	;
	
	// SPI Sensor pins
	SPI_SENSOR->MODER	|=
		(2 << (SPI_SENSOR_SCK * 2))		|
		(2 << (SPI_SENSOR_MISO * 2))	|
		(2 << (SPI_SENSOR_MOSI * 2))	;
	
	SPI_SENSOR->OSPEEDR |= 
		(2 << (SPI_SENSOR_SCK * 2))		|
		(2 << (SPI_SENSOR_MOSI * 2))	;
	
	SPI_SENSOR->AFR[0] |=
		(5 << (SPI_SENSOR_SCK * 4))		|
		(5 << (SPI_SENSOR_MISO * 4))	|
		(5 << (SPI_SENSOR_MOSI * 4))	;
	
	SPI_L3DG20->MODER	|=
		(1 << (SPI_CS_L3GD20 * 2))		;
		
	SPI_L3DG20->ODR |= (1 << SPI_CS_L3GD20);	
		
	SPI_ADNS5090->MODER |=
		(1 << (SPI_CS_ADNS5090 * 2))	;
	
	SPI_ADNS5090->ODR |= (1 << SPI_CS_ADNS5090);
		
	// ROS USART Pins
	RUART_PORT->MODER |=
		(2 << (RUART_TX * 2))		|
		(2 << (RUART_RX * 2))		;
	
	RUART_PORT->AFR[0] |=
		(7 << (RUART_TX * 4))		|
		(7 << (RUART_RX * 4))		;
		
	// Encoder Pin
	ENC1_PORT->MODER |=
		(2 << (ENC1_1_TIM3 * 2))	|
		(2 << (ENC1_2_TIM3 * 2))	;
	ENC1_PORT->AFR[0] |=
		(2 << (ENC1_1_TIM3 * 4))	|
		(2 << (ENC1_2_TIM3 * 4))	;
	
	ENC2_PORT->MODER |=
		(2 << (ENC2_1_TIM2 * 2))	|
		(2 << (ENC2_2_TIM2 * 2))	;
	ENC2_PORT->AFR[0] |=
		(2 << (ENC2_1_TIM2 * 4))	|
		(2 << (ENC2_2_TIM2 * 4))	;
		
	ENC3_PORT->MODER |=
		(2 << (ENC3_1_TIM8 * 2))	|
		(2 << (ENC3_2_TIM8 * 2))	;
	ENC3_PORT->AFR[0] |=
		(4 << (ENC3_1_TIM8 * 4))	|
		(4 << (ENC3_2_TIM8 * 4))	;
		
	ENC4_PORT->MODER |=
		(2 << (ENC4_1_TIM4 * 2))	|
		(2 << (ENC4_2_TIM4 * 2))	;
	ENC4_PORT->AFR[1] |=
		(2 << ((ENC4_1_TIM4 - 8) * 4))	|
		(2 << ((ENC4_2_TIM4 - 8) * 4))	;

	// Motor control pins
	PWM_PORT->MODER |=
		(2 << (M1_PWM * 2))		|
		(2 << (M2_PWm * 2))		|
		(2 << (M3_PWM * 2))		|
		(2 << (M4_PWM * 2))		;
	PWM_PORT->AFR[1] |=
		(2 << ((M1_PWM - 8) * 4))	|
		(2 << ((M2_PWm - 8) * 4))	|
		(2 << ((M3_PWM - 8) * 4))	|
		(2 << ((M4_PWM - 8) * 4))	;
	
	GPIOD->MODER |=
		(1 << (M2_INB * 2))	|
		(1 << (M4_INB * 2))	;
	GPIOB->MODER |=
		(1 << (M1_INB * 2))	;
	GPIOE->MODER |=
		(1 << (M3_INB * 2))	;
		
}

void app_robot_ioInit(){
	clk_init();
	app_robot_gpioInit();
	systick_init();
}

void app_robot_appInit(){
	// Initialize clock, systick and gpios
	app_robot_ioInit();
	
	// Initialize communication (UART)
	if(app_ros_comm_init(
		&odom_data_t,
		&imu_gyro_data_t,
		&imu_acmag_data_t,
		&motor_cmdvel_t,
		&motor_fbvel_t
	))
		while(1);
	
	// Initialize sensor communication (I2C, SPI)
	app_io_manager_init(
		&odom_data_t,
		&imu_gyro_data_t,
		&imu_acmag_data_t
	);
	
	// Initialize PWM and QDEC for motor control
	app_motor_control_init(
		&motor_cmdvel_t,
		&motor_fbvel_t
	);
	
}

void app_robot_statusRunner(){

}	
	
void app_robot_controlSysRunner(){
	if((millis() - controlRunner_millis) > LOOP_TIME_MOTORCTRL){
		controlRunner_millis = millis();
		app_motor_control_doEncoder();
		app_motor_control_doPID();
		app_motor_control_commandMotor();
	}
}

void app_robot_sensorRunner(){
	if((millis() - sensorRunner_millis) > LOOP_TIME_SENSOR){
		sensorRunner_millis = millis();
		// Check if we got all sensor data
		if(
			app_io_manager_getSPISensorDataReady() &&
			app_io_manager_getI2CSensorDataReady()
		){
		// trigger next measurement
			app_io_manager_SPITriggerNext();
			app_io_manager_I2CTriggerNext();
		}
	}
}

void app_robot_runner(){
	// Permanent, high priority runners
	app_io_manager_runner();
	app_robot_controlSysRunner();
	
	// High level, low priority runners
	app_ros_comm_runner();
	app_robot_sensorRunner();
	app_robot_statusRunner();
}