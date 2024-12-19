#define ROS_COMM_RX_SIZE    13
#define ROS_COMM_TX_SIZE    32

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
  COMMSTATE_WAITFORREPLY    = 0, // Wait for Header match, then process the message
  COMMSTATE_REPLYDATA       = 1, // Transmit the feedback message
};

// Private pointer
uint8_t *rx_ptr;
// Sensor data
sensor3dRaw_t    *imu_gyro_data_ptr_t;
sensor3dRaw_t    *imu_accel_data_ptr_t;
sensor3dRaw_t    *imu_mag_data_ptr_t;

// Motor data
wheelvel_t      *motor_cmdvel_ptr_t;
wheelvel_t      *motor_fbvel_ptr_t;

// Private variable
uint8_t comm_fsm        = 0;
uint8_t new_input_data  = 0;

uint8_t rx_fsm          = 0;
uint8_t rx_pointer_idx  = 0;
uint8_t rx_data_max_cnt = 0;
uint8_t rx_timeOutFlag = 0;

uint8_t tx_fsm = 0;
uint8_t tx_pointer_idx = 0;
uint8_t tx_done = 0;

uint8_t prevWriteAvailable = 0;

// Private typedefs
ros_rbc_ioPacket_t  rbc_Packet_t;

void app_ros_comm_init(
  sensor3dRaw_t       *gyro_ptr_t,
  sensor3dRaw_t       *accel_ptr_t,
  sensor3dRaw_t       *mag_ptr_t,
  wheelvel_t          *m_cmdvel_ptr_t,
  wheelvel_t          *m_fbvel_ptr_t
  ){
  Serial.begin(230400);  
  rx_ptr = (uint8_t *)&rbc_Packet_t;

  imu_gyro_data_ptr_t   = gyro_ptr_t;
  imu_accel_data_ptr_t  = accel_ptr_t;
  imu_mag_data_ptr_t    = mag_ptr_t;
  motor_cmdvel_ptr_t    = m_cmdvel_ptr_t;
  motor_fbvel_ptr_t     = m_fbvel_ptr_t;
}

void app_ros_comm_rxPoll(){
  if(Serial.available() < 1)
    return;  

  new_input_data = Serial.read();
  
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
      rx_fsm = 0;
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

    case 1:// Write data to TX buffer
    {
      Serial.write(
        (uint8_t *)(&rbc_Packet_t.ajbHeader),
        ROS_COMM_TX_SIZE
      ); 

      tx_fsm = 2;
    }
    break;

    case 2:
    {
      if(Serial.availableForWrite() < 128)
        break;

      tx_done = 1;  
      tx_fsm = 0;
    }
    break;
  
    default:
      tx_fsm = 0;
      break;
  }
  
}
void app_ros_comm_runner(){

  if((millis() - commTimeout_millis) > COMM_TIMEOUT_MS){
    commTimeout_millis = millis(); 
    comm_fsm = COMMSTATE_WAITFORREPLY;
    rx_timeOutFlag = 1;
    tx_fsm = 0;
    tx_pointer_idx = 0;
    rx_fsm = 0;
    motor_cmdvel_ptr_t->v1 = 0.0f;
    motor_cmdvel_ptr_t->v2 = 0.0f;
    motor_cmdvel_ptr_t->v3 = 0.0f;
    motor_cmdvel_ptr_t->v4 = 0.0f;
  } 
  
  switch(comm_fsm){
    case COMMSTATE_WAITFORREPLY:
    {
      app_ros_comm_rxPoll();
      if(rx_fsm != 127)
        break;

      commTimeout_millis = millis();  
      rx_timeOutFlag = 0;
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
  (rbc_Packet_t.rbcHeader[0] != 'R')  ||
  (rbc_Packet_t.rbcHeader[1] != 'B')
  )
    return;

  // Apply header
  rbc_Packet_t.ajbHeader[0] = 'J';
  rbc_Packet_t.ajbHeader[1] = 'B';  
  
  // Apply motor RPM command
  motor_cmdvel_ptr_t->v1  = 
    (float)rbc_Packet_t.motorControl.motor1_ctrl;// LF
  motor_cmdvel_ptr_t->v2  =
    (float)rbc_Packet_t.motorControl.motor2_ctrl;// LB
  motor_cmdvel_ptr_t->v3  =
    (float)rbc_Packet_t.motorControl.motor3_ctrl;// RB
  motor_cmdvel_ptr_t->v4  =
    (float)rbc_Packet_t.motorControl.motor4_ctrl;// RF

  // Process PC command
  app_ros_comm_comandProcessor();  
  
  // Send encoder RPM count
  rbc_Packet_t.motorFeedBack.motor1_fb  =
    motor_fbvel_ptr_t->v1;                       // LF
  rbc_Packet_t.motorFeedBack.motor2_fb  =
    motor_fbvel_ptr_t->v2;                       // LB
  rbc_Packet_t.motorFeedBack.motor3_fb  =
    motor_fbvel_ptr_t->v3;                       // RB
  rbc_Packet_t.motorFeedBack.motor4_fb  =
    motor_fbvel_ptr_t->v4;                       // RF
    
  // Send mouse data
  
  
  // Send gyro data
  rbc_Packet_t.gyro_x_raw =
    imu_gyro_data_ptr_t->sx.sx16;
  rbc_Packet_t.gyro_y_raw =
    imu_gyro_data_ptr_t->sy.sy16;
  rbc_Packet_t.gyro_z_raw =
    imu_gyro_data_ptr_t->sz.sz16;
  
  // Send mag data
  rbc_Packet_t.mag_x_raw  =
    imu_mag_data_ptr_t->sx.sx16;
  rbc_Packet_t.mag_y_raw  =
    imu_mag_data_ptr_t->sy.sy16;
  rbc_Packet_t.mag_z_raw  =
    imu_mag_data_ptr_t->sz.sz16;
  
  // Send accel data
  rbc_Packet_t.acc_x_raw  =
    imu_accel_data_ptr_t->sx.sx16;
  rbc_Packet_t.acc_y_raw  =
    imu_accel_data_ptr_t->sy.sy16;
  rbc_Packet_t.acc_z_raw  =
    imu_accel_data_ptr_t->sz.sz16;
    
  comm_fsm = COMMSTATE_REPLYDATA;
  tx_fsm = 1;
}

void app_ros_comm_comandProcessor(){
  switch(rbc_Packet_t.reg & 0x7F){
    case 0x00:// Null command
      {
        // Reply command
        rbc_Packet_t.cmdDataMCU = 0x00;
      }
      break;

    case 0xAA:// Emergency stop
      {
        // Reply command
        rbc_Packet_t.cmdDataMCU = 0x55;
        motor_cmdvel_ptr_t->v1  = 0.0f;
        motor_cmdvel_ptr_t->v2  = 0.0f;
        motor_cmdvel_ptr_t->v3  = 0.0f;
        motor_cmdvel_ptr_t->v4  = 0.0f;
        motor_eStop();
      }
      break;
      
    default:
        // Reply command
        rbc_Packet_t.cmdDataMCU = 0x00;
      break;
  }
 
}

uint8_t app_ros_comm_TxStarted(){
  return (COMMSTATE_REPLYDATA == comm_fsm) ? 1 : 0;  
}

uint8_t app_ros_comm_TxDone(){
  if(tx_done == 0)
    return 0;

  tx_done = 0;
  return 1;
}

uint8_t app_ros_comm_timeOut(){
  if(rx_timeOutFlag == 0)
    return 0;

  rx_timeOutFlag = 0;
  return 1;
}
