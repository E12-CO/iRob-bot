#ifndef STD_ODOMETRY_H
#define STD_ODOMETRY_H

#ifdef __cplusplus
extern "C"{
#endif

// Robot odometry struct
// User need to update following variables
// pos_x, pos_y (velocity integration or optical flow position integration)
// pos_az (from kinematic's velocity integration, IMU or magnetometer)
// 
// vel_x, vel_y (velocity sensed from encoder or optical flow calculated velocity)
// vel_az (from kinematic's velocity, IMU's gyro or magnetometer differentiation)

typedef struct {
  // Current position
  float pos_x;// Linear X position (m)
  float pos_y;// Linear Y position (m)
  float pos_az;// Angular Z position (rad)
  // Previous position
  float prev_pos_x;
  float prev_pos_y;
  float prev_pos_az;

  // Current velocity
  float vel_x;// Linear X velocity (m/s)
  float vel_y;// Linear Y velocity (m/s)
  float vel_az;// Angular Z velocity (rad/s)
  // Previous velocity
  float prev_vel_x;
  float prev_vel_y;
  float prev_vel_az;

  // Current velocity heading
  float pos_heading;// Angular Z heading (rad)

  float pos_abs_az;// Absolute angular z position
}odometry_t;

typedef struct {
  float vlx_out;// Linear X velocity output
  float vly_out;// Linear Y velocity output
  float vaz_out;// Angular Z velocity output
}cmdvel_t;

typedef struct{
  float vel;// Distance
  float heading;// Heading
  float vaz; // Angular velocity  
}headvel_t;

typedef struct {
  float v1;// Front wheel angular velocity (rad/s)
  float v2;// Left wheel angular velocity (rad/s)
  float v3;// Right wheel angular velocity (rad/s)
}wheelvel_t;

typedef struct{
  float motor_Kp;
  float motor_Ki;

  float curr_speed;
  float prev_speed;
  
  float e_speed;
  float Intg_e_speed;

  int speed_cmd;
}motor_var_t;

#ifdef __cplusplus
}
#endif

#endif
