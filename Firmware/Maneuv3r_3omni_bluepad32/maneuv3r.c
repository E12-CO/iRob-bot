#include "maneuv3r.h"

// Private pointer
odometry_t *robot_odom;
cmdvel_t *robot_cmd;

// Private variables
uint16_t goal_timeout = 0 ;

// Private typedef
pidConst_t *pidptr_walk_t;// PID constant for linear motion
pidConst_t *pidptr_rotate_t;// PID constant for angular motion
pidConst_t *pidptr_twizzles_t;// PID constant for twizzles rotate control

maneuv3r_joyttracker_t joytracker_t;

// special function
float atan2pi(float y, float x) {
  float at = atan2f(y, x);
  if (at < 0.0f)
    at += 6.28319f;
  return at;
}

void maneuv3r_init(
  odometry_t *robot_odom_ptr,
  cmdvel_t *robot_cmd_ptr,
  pidConst_t *ptr_pid_walk_t,
  pidConst_t *ptr_pid_rotate_t,
  pidConst_t *ptr_pid_twizzles_t
  ){

  robot_odom  = robot_odom_ptr;
  robot_cmd   = robot_cmd_ptr;

  pidptr_walk_t        = ptr_pid_walk_t;
  pidptr_rotate_t      = ptr_pid_rotate_t;
  pidptr_twizzles_t    = ptr_pid_twizzles_t;
    
//  robot_odom->pos_x       = 0.0;
//  robot_odom->pos_y       = 0.0;
//  robot_odom->pos_az      = 0.0;
//  robot_odom->prev_pos_x  = 0.0;
//  robot_odom->prev_pos_y  = 0.0;
//  robot_odom->prev_pos_az = 0.0;
//
//  robot_odom->vel_x       = 0.0;
//  robot_odom->vel_y       = 0.0;
//  robot_odom->vel_az      = 0.0;
//  robot_odom->prev_vel_x  = 0.0;
//  robot_odom->prev_vel_y  = 0.0;
//  robot_odom->prev_vel_az = 0.0;
//  
//  robot_odom->pos_heading = 0.0;
}


void maneuv3r_update_Cmdvel(
  float vel, 
  float heading, 
  float az) {
  // Convert the R and Theta (polar coordinates) into X and Y velocity component (Cartesian coordinates).
  robot_cmd->vlx_out = vel * cosf(heading);
  robot_cmd->vly_out = vel * sinf(heading);

  // Commading angular velocity.
  robot_cmd->vaz_out = az;
}

void maneuv3r_joyTracker(
  float vel, 
  float heading, 
  float rotate, 
  uint16_t joy_cmd){
  goal_timeout ++;

  // Gas paddle (R2)
  if(joy_cmd & 0x8000){
    goal_timeout = 0;
    vel *= 1.5f;  
    rotate *= 1.3f;
  }

  joytracker_t.out_vel = vel; 
    
  // Velocity Dead band
  if((joytracker_t.out_vel > -0.005f) && 
    (joytracker_t.out_vel < 0.005f))
    joytracker_t.out_vel = 0.0f;
  
  // Calculate position setpoint form Joy stick vel command integration
  joytracker_t.dest_x += (vel * cosf(heading)) * 0.008f;
  joytracker_t.dest_y += (vel * sinf(heading)) * 0.008f;

  // Integrate cmd_vel to position for position control.
  joytracker_t.pos_az_cmd += rotate * 0.008f;
  
  if((joytracker_t.cmd_lock == 0) && (joy_cmd & 0x7FFF)){
    switch(joy_cmd){
      case 0x0080:// Set new north (Y)
      {
        joytracker_t.pos_az_cmd = 0.0f;
        robot_odom->pos_abs_az = 0.0f;
        robot_odom->pos_az = 0.0f; 
        joytracker_t.Intg_e_orient = 0.0f;
      }
      break;

      case 0x0010:// Rotate to north (A)
      {
       if(robot_odom->pos_abs_az < 3.141593f)
        joytracker_t.pos_az_cmd -= robot_odom->pos_abs_az;
       else
        joytracker_t.pos_az_cmd += 6.28319f - robot_odom->pos_abs_az;
        
      }
      break;

      case 0x0200:// rotate cw (relative to bot) (R1)
      {
        joytracker_t.pos_az_cmd -= 1.570796f;
      }
      break;

      case 0x0100:// rotate ccw (relative to bot) (L1)
      {
        joytracker_t.pos_az_cmd += 1.570796f;
      }
      break;

      default:
        break;
    }
        
    joytracker_t.cmd_lock = 1;
  }else if((joytracker_t.cmd_lock == 1) && !(joy_cmd & 0x7FFF))
    joytracker_t.cmd_lock = 0;


   /*========================== BEGIN LINEAR VELOCITY===================================*/
    // Calculate the current distance (R) in polar coordinates
    joytracker_t.diff_x = joytracker_t.dest_x - robot_odom->pos_x;// SV - PV
    joytracker_t.diff_y = joytracker_t.dest_y - robot_odom->pos_y;// SV - PV

    /*============================ BEGIN VELOCITY HEADING ================================*/
    // Maintain heading to the destination
    joytracker_t.cmd_heading = atan2pi(
       joytracker_t.diff_y, 
       joytracker_t.diff_x);// Calculate the heading
    /*============================== END VELOCITY HEADING ================================*/
    
    // Take a square
    joytracker_t.diff_x = joytracker_t.diff_x * joytracker_t.diff_x;// diff_x^2
    joytracker_t.diff_y = joytracker_t.diff_y * joytracker_t.diff_y;// diff_y^2
    // Claculate distance error e_dist(z)
    // e_dist(z) = sqrt( (dest_x - pos_x)^2 + (dest_y - pos_y)^2 );
    joytracker_t.e_dist = sqrtf(joytracker_t.diff_x + joytracker_t.diff_y);// Euclidean distance
  
    // PID controller algorithm for distance tracking
    joytracker_t.Intg_e_dist += joytracker_t.e_dist * pidptr_walk_t->Ki;// I term
  
    joytracker_t.Diff_e_dist =
      (joytracker_t.e_dist - joytracker_t.prev_e_dist)
      * pidptr_walk_t->Kd;                                                // D term
    joytracker_t.prev_e_dist = joytracker_t.e_dist;                 // make a unit delay
  
    // Finally calculate the lienar velocity CV (command value)
    joytracker_t.cmd_lvel =
      (joytracker_t.e_dist * pidptr_walk_t->Kp) +
      joytracker_t.Intg_e_dist +
      joytracker_t.Diff_e_dist;

    // Dead band
    if((joytracker_t.cmd_lvel < pidptr_walk_t->min_vlin) && (joytracker_t.cmd_lvel > -pidptr_walk_t->min_vlin))
      joytracker_t.cmd_lvel = 0.0f;
    /*============================== END LINEAR VELOCITY  ================================*/


  /*============================== BEGIN ORIENTATION ================================*/
  joytracker_t.e_orient = (joytracker_t.pos_az_cmd - robot_odom->pos_az);
  joytracker_t.Intg_e_orient += joytracker_t.e_orient * 0.0003f;

  joytracker_t.cmd_avel = 
     (joytracker_t.e_orient * 5.0f) +
      joytracker_t.Intg_e_orient;
  /*============================== END ORIENTATION ================================*/

  // Once all goals were satisfied, stop the robot
  if((joytracker_t.e_dist > -DISTANCE_TOL) &&
     (joytracker_t.e_dist < DISTANCE_TOL)
    ) {
    goal_timeout = 0;
    joytracker_t.cmd_lvel = 0.0f;
    joytracker_t.Intg_e_dist = 0.0f;
  }

  if((joytracker_t.e_orient > -ROTATION_TOL) &&
     (joytracker_t.e_orient < ROTATION_TOL)){
    goal_timeout = 0;
    joytracker_t.cmd_avel = 0.0f;
    joytracker_t.Intg_e_orient = 0.0f;
  }

  if(goal_timeout > GOAL_TIMEOUT){
    goal_timeout = 0;
    // Update SP
    joytracker_t.dest_x = robot_odom->pos_x;
    joytracker_t.dest_y = robot_odom->pos_y;
    joytracker_t.pos_az_cmd = robot_odom->pos_az;

    // Reset PID
    joytracker_t.cmd_avel = 0.0f;
    joytracker_t.Intg_e_orient = 0.0f;
    joytracker_t.cmd_lvel = 0.0f;
    joytracker_t.Intg_e_dist = 0.0f;
  }
  
  maneuv3r_update_Cmdvel(
    joytracker_t.cmd_lvel, 
    joytracker_t.cmd_heading - robot_odom->pos_abs_az, 
    joytracker_t.cmd_avel);  
}
