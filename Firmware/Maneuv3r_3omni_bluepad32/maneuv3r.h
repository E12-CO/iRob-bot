#ifndef MANEUV3R_H
#define NANEUV3R_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdint.h>
#include "stdodom.h"

// Macros
#define DISTANCE_TOL 0.01f
#define ROTATION_TOL 0.015f

#define GOAL_TIMEOUT  250   // 250  * 0.008 = 2 second

// Typedefs
// PID typedef
typedef struct {
  float Kp;
  float Ki;
  float Kd;

  // Maximum and Minimum linear velocity
  float min_vlin;
  float max_vlin;
  // Maximum and Minimum linear velocity
  float min_vang;
  float max_vang;
} pidConst_t;

// JoyTracker
typedef struct {
  float next_vel;
  float out_vel;
  
  float pos_az_cmd;
  
  uint8_t cmd_lock;

  /*=============== Translation =====================*/
  // Calculate the X and Y distance from current position to the destination
  float dest_x;
  float dest_y;

  float cmd_lvel;

  // Calculate the difference between current position and destination.
  float diff_x;
  float diff_y;
  float e_dist;// Distance error (SV - PV)
  float Intg_e_dist;// Integral accumulator of the e_dist
  float Diff_e_dist;// Differentiator output
  float prev_e_dist;// z-1 of the e_dist

  /*================ velocity heading =============*/
  float cmd_heading;

  /*================ Rotation ====================*/
  float cmd_avel;
  float e_orient;// Orientation error (SV - PV)

  float Intg_e_orient;// Integral accumulator of the e_orient
}maneuv3r_joyttracker_t;

extern maneuv3r_joyttracker_t joytracker_t;

float atan2pi(float y, float x);

void maneuv3r_init(
  odometry_t *robot_odom_ptr,
  cmdvel_t *robot_cmd_ptr,
  pidConst_t *ptr_pid_walkt_t,
  pidConst_t *ptr_pid_rotate_t,
  pidConst_t *ptr_pid_twizzles_t
);

void maneuv3r_update_Cmdvel(
  float vel,
  float heading,
  float az);

// For auto control mode


// For manual control mode
inline void maneuv3r_joyTracker(
  float vel, 
  float heading, 
  float rotate, 
  uint16_t joy_cmd);


#ifdef __cplusplus
}
#endif

#endif
