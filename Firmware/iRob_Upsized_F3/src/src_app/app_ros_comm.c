// Communication handling between the MCU and ROS node on PC
#include "app_ros_comm.h"

#define ROS_COMM_RX_SIZE    13
#define ROS_COMM_TX_SIZE    32

// Private pointers
uint8_t *rx_ptr;
// Sensor data
adns_data_t 			*odom_data_ptr_t;			// ADNS-5090 mouse odometry
l3gd20_data_t			*imu_gyro_data_ptr_t;	// L3GD20 Gyro data
lsm303dlhc_data_t	*imu_acmag_data_ptr_t;	// Accel and Mag data

// Motor data
motor_rpm_t 		*motor_cmdvel_ptr_t; // Command velocity for motor
motor_rpm_t			*motor_fbvel_ptr_t;	// Encoder feedback	

// Private variables
uint8_t comm_fsm = 0;

volatile uint8_t rx_fsm = 0;
volatile uint8_t rx_pointer_idx  = 0;
volatile uint8_t rx_data_max_cnt = 0;

uint8_t tx_fsm = 0;
uint8_t tx_pointer_idx = 0;

// Private typedefs
ros_rbc_ioPacket_t	rbc_Packet_t;

// Private prototypes
void app_ros_comm_rxCallback(uint8_t new_input_data);
void app_ros_comm_processRX();

uint8_t app_ros_comm_init(
	adns_data_t 			*adns_ptr_t,
	l3gd20_data_t			*gyro_ptr_t,
	lsm303dlhc_data_t	*acmag_ptr_t,
	motor_rpm_t				*m_cmdvel_ptr_t,
	motor_rpm_t				*m_fbvel_ptr_t
	){
	if(ROSusart_init(
		BAUD_230400,						// Using baud rate of 230.4kBaud
		app_ros_comm_rxCallback	// Registering the rx call back
		))
		return 1;// In case of USART init error, return 1
	
	rx_ptr = (uint8_t *)&rbc_Packet_t;
	
	if(adns_ptr_t == 0)
		return 1;
	
	if(gyro_ptr_t == 0)
		return 1;
	
	if(acmag_ptr_t == 0)
		return 1;
	
	if(m_cmdvel_ptr_t == 0)
		return 1;
	
	if(m_fbvel_ptr_t == 0)
		return 1;
	
	odom_data_ptr_t 			= adns_ptr_t;
	imu_gyro_data_ptr_t		=	gyro_ptr_t;
	imu_acmag_data_ptr_t	= acmag_ptr_t;
	motor_cmdvel_ptr_t		=	m_cmdvel_ptr_t;
	motor_fbvel_ptr_t			=	m_fbvel_ptr_t;
	
	return 0;
}

void app_ros_comm_rxCallback(uint8_t new_input_data){
	
	switch(rx_fsm){
		case 0:// Wait for 'R' in the header
		{
			if(new_input_data == 'R'){
				*(rx_ptr + rx_pointer_idx) = new_input_data;
				rx_pointer_idx++;
				rx_fsm = 1;
			}
		}
		break;
		
		case 1:// Wait for 'B' in the header
		{
			if(new_input_data == 'B'){
				*(rx_ptr + rx_pointer_idx) = new_input_data;
				rx_pointer_idx++;
				rx_fsm = 2;
			}
		}
		break;

		case 2:// Get the reg byte and do read/write check
		{
			*(rx_ptr + rx_pointer_idx) = new_input_data;
			
			if(rbc_Packet_t.regBit.rw)// 1- > Read request
					rx_data_max_cnt = 9;
			else
					rx_data_max_cnt = 10;
			
			rx_pointer_idx++;
			rx_fsm = 3;
		}
		break;
		
		case 3:// Free-running rx until we got all bytes
		{
			*(rx_ptr + rx_pointer_idx) = new_input_data;
			rx_pointer_idx++;
			
			if(rx_pointer_idx == ROS_COMM_RX_SIZE){
				rx_pointer_idx = 0;
				rx_fsm = 127;
			}
			
		}
		break;
		
		case 127:// halt case
		{
		
		}
		break;
		
	}
	
}

void app_ros_comm_txPoll(){
	
	switch(tx_fsm){
		case 0:// Idle state
		{
		
		}
		break;
	
		case 1:// Check if UART is ready to TX
		{
			if(ROSusart_checkTxEmpty()){
				tx_fsm = 2;
			}
		}
		break;
		
		case 2:// Write byte to UART
		{
			ROSusart_transmitData(
				*((uint8_t *)(&rbc_Packet_t.cmdDataMCU) + 
					tx_pointer_idx)
			);
			
			tx_pointer_idx++;
			
			tx_fsm = 3;
		}
		break;
		
		case 3:// Check Transfer complete
		{
			if(!ROSusart_checkTxComplete())
				return;
			
			if(tx_pointer_idx == ROS_COMM_TX_SIZE){
				tx_pointer_idx = 0;
				tx_fsm = 0;
			}else{
				tx_fsm = 2;
			}
			
		}
		break;
		
		default:
			tx_fsm = 0;
			break;
	}
	
}

void app_ros_comm_runner(){
	
	switch(comm_fsm){
		case COMMSTATE_WAITFORREPLY:
		{
			if(rx_fsm != 127)
				break;
			
			rx_fsm = 0;// reset fsm state 
			// Process RX data here
			app_ros_comm_processRX();
			
			
		}
		break;
		
		case COMMSTATE_REPLYDATA:
		{
			app_ros_comm_txPoll();
			if(tx_fsm == 0)
				comm_fsm = COMMSTATE_WAITFORREPLY;
		}
		break;
	}

}

void app_ros_comm_processRX(){
	if(
	(rbc_Packet_t.rbcHeader[0] != 'R')	||
	(rbc_Packet_t.rbcHeader[1] != 'B')
	)
		return;
	
	// Apply header
	rbc_Packet_t.ajbHeader[0] = 'J';
  rbc_Packet_t.ajbHeader[1] = 'B'; 
	
	// Apply motor RPM command
	motor_cmdvel_ptr_t->m1_rpm 	= 
		(float)rbc_Packet_t.motorControl.motor1_ctrl;
	motor_cmdvel_ptr_t->m2_rpm	=
		(float)rbc_Packet_t.motorControl.motor2_ctrl;
	motor_cmdvel_ptr_t->m3_rpm	=
		(float)rbc_Packet_t.motorControl.motor3_ctrl;
	motor_cmdvel_ptr_t->m4_rpm	=
		(float)rbc_Packet_t.motorControl.motor4_ctrl;
	
	// Reply command
	rbc_Packet_t.cmdDataMCU = 0x00;
	
	// Send encoder RPM count
	rbc_Packet_t.motorFeedBack.motor1_fb	=
		motor_fbvel_ptr_t->m1_rpm;
	rbc_Packet_t.motorFeedBack.motor2_fb	=
		motor_fbvel_ptr_t->m2_rpm;
	rbc_Packet_t.motorFeedBack.motor3_fb	=
		motor_fbvel_ptr_t->m3_rpm;
	rbc_Packet_t.motorFeedBack.motor4_fb	=
		motor_fbvel_ptr_t->m4_rpm;
		
	// Send mouse data
	
	
	
	// Send gyro data
	rbc_Packet_t.gyro_x_raw	=
		imu_gyro_data_ptr_t->gyro_x;
	rbc_Packet_t.gyro_y_raw	=
		imu_gyro_data_ptr_t->gyro_y;
	rbc_Packet_t.gyro_z_raw	=
		imu_gyro_data_ptr_t->gyro_z;
	
	// Send mag data
	rbc_Packet_t.mag_x_raw	=
		imu_acmag_data_ptr_t->mag_x;
	rbc_Packet_t.mag_y_raw	=
		imu_acmag_data_ptr_t->mag_y;
	rbc_Packet_t.mag_z_raw	=
		imu_acmag_data_ptr_t->mag_z;
	
	// Send accel data
	rbc_Packet_t.acc_x_raw	=
		imu_acmag_data_ptr_t->acc_x;
	rbc_Packet_t.acc_y_raw	=
		imu_acmag_data_ptr_t->acc_y;
	rbc_Packet_t.acc_z_raw	=
		imu_acmag_data_ptr_t->acc_z;
		
	comm_fsm = COMMSTATE_REPLYDATA;
	tx_fsm = 1;
}