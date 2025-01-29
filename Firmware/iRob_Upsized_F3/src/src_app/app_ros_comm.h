#ifndef APP_ROS_COMM_H
#define APP_ROS_COMM_H

#include "ros_usart.h"

#include "app_io_manager.h"
#include "app_motor_control.h"

// Typedef for communication
typedef struct __attribute__((packed)){
  // Send by PC
  uint8_t rbcHeader[2];
  
  union{
    uint8_t reg;
    struct{
      uint8_t address   :5;
      uint8_t type      :2;
      uint8_t rw        :1;
    }regBit;    
  };
	
  uint8_t CTK;
  
  struct{
    int16_t motor1_ctrl;
    int16_t motor2_ctrl;
    int16_t motor3_ctrl;
    int16_t motor4_ctrl;
  }motorControl;

  uint8_t cmdDataPC;
  
  // Receive from MCU
  uint8_t ajbHeader[2];
  uint8_t cmdDataMCU;

  struct{
    int16_t motor1_fb;
    int16_t motor2_fb;
    int16_t motor3_fb;
    int16_t motor4_fb;
  }motorFeedBack;
  
  struct{
    int8_t mouse_x_vel;
    int8_t mouse_y_vel;
  }mouseVel;
  
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;
  
  int16_t mag_x_raw;
  int16_t mag_y_raw;
  int16_t mag_z_raw;
  
  int16_t acc_x_raw;
  int16_t acc_y_raw;
  int16_t acc_z_raw;
  
  uint8_t cks;
}ros_rbc_ioPacket_t;


// state machine enum
enum ROS_COMM_FSM{
	COMMSTATE_WAITFORREPLY 		= 0, // Wait for Header match, then process the message
	COMMSTATE_REPLYDATA				= 1, // Transmit the feedback message
};

uint8_t app_ros_comm_init(
	adns_data_t 			*adns_ptr_t,
	l3gd20_data_t			*gyro_ptr_t,
	lsm303dlhc_data_t	*acmag_ptr_t,
	motor_rpm_t				*m_cmdvel_ptr_t,
	motor_rpm_t				*m_fbvel_ptr_t
	);
void app_ros_comm_runner();

#endif