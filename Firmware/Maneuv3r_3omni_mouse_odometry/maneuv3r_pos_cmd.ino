// TODO : use fixed-point math for speed and resource.

#define DISTANCE_TOL 0.008f
#define ROTATION_TOL 0.035f

pidConst_t pid_walk_t;// PID constant for linear motion
pidConst_t pid_twizzles_t;// PID constant for twizzles rotate control
pidConst_t pid_rotate_t;// PID constant for angular motion

odometry_t *robot_odom;
cmdvel_t *robot_cmd;

void maneuv3r_init(
  odometry_t *robot_odom_ptr,
  cmdvel_t *robot_cmd_ptr
  ){

  robot_odom = robot_odom_ptr;
  robot_cmd = robot_cmd_ptr;
    
  robot_odom->pos_x = 0.0;
  robot_odom->pos_y = 0.0;
  robot_odom->pos_az = 0.0;
  robot_odom->prev_pos_x = 0.0;
  robot_odom->prev_pos_y = 0.0;
  robot_odom->prev_pos_az = 0.0;

  robot_odom->vel_x = 0.0;
  robot_odom->vel_y = 0.0;
  robot_odom->vel_az = 0.0;
  robot_odom->prev_vel_x = 0.0;
  robot_odom->prev_vel_y = 0.0;
  robot_odom->prev_vel_az = 0.0;
  
  robot_odom->pos_heading = 0.0;
}

void maneuv3r_pidInit(
  float Kp_walk,
  float Ki_walk,
  float Kd_walk,
  float min_walk,
  float max_walk,

  float Kp_twizzles,
  float Ki_twizzles,
  float Kd_twizzles,

  float Kp_rotate,
  float Ki_rotate,
  float Kd_rotate,
  float min_rotate,
  float max_rotate) {

  pid_walk_t.Kp = Kp_walk;
  pid_walk_t.Ki = Ki_walk;
  pid_walk_t.Kd = Kd_walk;
  pid_walk_t.min_vlin = min_walk;
  pid_walk_t.max_vlin = max_walk;

  pid_twizzles_t.Kp =  Kp_twizzles;
  pid_twizzles_t.Ki =  Ki_twizzles;
  pid_twizzles_t.Kd =  Kd_twizzles;

  pid_rotate_t.Kp = Kp_rotate;
  pid_rotate_t.Ki = Ki_rotate;
  pid_rotate_t.Kd = Kd_rotate;
  pid_rotate_t.min_vang = min_rotate;
  pid_rotate_t.max_vang = max_rotate;
}

void maneuv3r_update_Cmdvel(float vel, float heading, float az) {
  // Convert the R and Theta (polar coordinates) into X and Y velocity component (Cartesian coordinates).
  robot_cmd->vlx_out = vel * cos(heading);
  robot_cmd->vly_out = vel * sin(heading);

  // Commading angular velocity.
  robot_cmd->vaz_out = az;
}

uint8_t maneuv3r_walkTracker_FSM = 0;// Tracking's state machine for normal walk.
typedef struct maneuv3r_walktracker_t {
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

  float cmd_avel;
  float initial_orient;
  float e_orient;// Orientation error (SV - PV)

  float Intg_e_orient;// Integral accumulator of the e_orient
  float Diff_e_orient;// Differentiator output
  float prev_e_orient;// z-1 of the e_orient

  float cmd_heading;
};
maneuv3r_walktracker_t walktracker_t;
// PID path tracking function that allows the robot to move in any xy floor plane direction by specifying :
// dist : moving distance
// heading : heading angle (moving direction)
uint8_t maneuv3r_walkTracker(float dist, float heading) {

  switch (maneuv3r_walkTracker_FSM) {
    case 0:// Setup state
      {
        // Clear previous calculations
        walktracker_t.Intg_e_dist = 0.0;
        walktracker_t.Intg_e_orient = 0.0;
        
        // Destination setpoint
        // The heading angle from user input will only be used for calculating the initial condition.
        walktracker_t.dest_x = (dist * cos(heading)) + robot_odom->pos_x;
        walktracker_t.dest_y = (dist * sin(heading)) + robot_odom->pos_y;
        // Setpoint for maintaining steady orientation
        // This helps stabilize robot when moving along Y axis
        walktracker_t.initial_orient = robot_odom->pos_az;
        
        maneuv3r_walkTracker_FSM = 1;
      }
      break;
      
    case 1:// Tracking state
      {
        /*===================== BEGIN LINEAR VELOCITY===================================*/
        // Claculate distance error e_dist(z)
        // e_dist(z) = sqrt( (dest_x - pos_x)^2 + (dest_y - pos_y)^2 );
        // Calculate the current distance (R) in polar coordinates
        walktracker_t.diff_x = walktracker_t.dest_x - robot_odom->pos_x;// SV - PV
        walktracker_t.diff_y = walktracker_t.dest_y - robot_odom->pos_y;// SV - PV


        /*============================ BEGIN VELOCITY HEADING ================================*/
        // Maintain heading to the destination
        walktracker_t.cmd_heading = atan2pi(walktracker_t.diff_y, walktracker_t.diff_x);// Calculate the heading
        /*============================== END VELOCITY HEADING ================================*/
        
        // Take a square
        walktracker_t.diff_x = walktracker_t.diff_x * walktracker_t.diff_x;// diff_x^2
        walktracker_t.diff_y = walktracker_t.diff_y * walktracker_t.diff_y;// diff_y^2
        // Calculate the Euclidean distance
        walktracker_t.e_dist = sqrt(walktracker_t.diff_x + walktracker_t.diff_y);
  
        // PID controller algorithm for distance tracking
        walktracker_t.Intg_e_dist += walktracker_t.e_dist * pid_walk_t.Ki;// I term
  
        walktracker_t.Diff_e_dist =
          (walktracker_t.e_dist - walktracker_t.prev_e_dist)
          * pid_walk_t.Kd;                                                // D term
        walktracker_t.prev_e_dist = walktracker_t.e_dist;                 // make a unit delay
  
        // Finally calculate the lienar velocity CV (command value)
        walktracker_t.cmd_lvel =
          (walktracker_t.e_dist * pid_walk_t.Kp) +
          walktracker_t.Intg_e_dist +
          walktracker_t.Diff_e_dist;
          
        // Velocity Envelope
        // Max envelope
        if(walktracker_t.cmd_lvel > pid_walk_t.max_vlin)
          walktracker_t.cmd_lvel = pid_walk_t.max_vlin;
        else if(walktracker_t.cmd_lvel < -pid_walk_t.max_vlin)
          walktracker_t.cmd_lvel = -pid_walk_t.max_vlin;

        // Dead band
//        if((walktracker_t.cmd_lvel < pid_walk_t.min_vlin) && (walktracker_t.cmd_lvel > -pid_walk_t.min_vlin))
//          walktracker_t.cmd_lvel = 0.0;
        /*============================= END LINEAR VELOCITY===================================*/

        /*============================== BEGIN ORIENTATION ================================*/
        // Calculate the orientation error e_orient(z), The set point is the initial angular position (Keeping robot facing the same direction).
        walktracker_t.e_orient = walktracker_t.initial_orient - robot_odom->pos_az;// pos_az is highly recommended to derive from either Gyro, Mag or fusion of both
      
        // PID controller algorithm for rotating
        walktracker_t.Intg_e_orient += walktracker_t.e_orient * pid_rotate_t.Ki;// I term
        walktracker_t.Diff_e_orient =
          (walktracker_t.e_orient - walktracker_t.prev_e_orient)
          * pid_rotate_t.Kd;                                                          // D term
        walktracker_t.prev_e_orient = walktracker_t.e_orient;                 // make a unit delay
      
        walktracker_t.cmd_avel =
          (walktracker_t.e_orient * pid_rotate_t.Kp) +
          walktracker_t.Intg_e_orient +
          walktracker_t.Diff_e_orient;

        // Angular velocity envelope
        // Max envelope
           walktracker_t.cmd_avel = 
              constrain(
                walktracker_t.cmd_avel,
                -pid_rotate_t.max_vang,
                pid_rotate_t.max_vang
              );
        // Dead band
        if((walktracker_t.cmd_avel < pid_rotate_t.min_vang) && (walktracker_t.cmd_avel > -pid_rotate_t.min_vang))
          walktracker_t.cmd_avel = 0.0;
        /*============================== END ORIENTATION ================================*/

        // Goal checker
        if (abs(walktracker_t.e_dist) < DISTANCE_TOL){ // meets the distance tolerance
          maneuv3r_update_Cmdvel(0.0, 0.0, 0.0);
          maneuv3r_walkTracker_FSM = 0;// Exit
          return 1;
        }
        
        maneuv3r_update_Cmdvel(walktracker_t.cmd_lvel, walktracker_t.cmd_heading, walktracker_t.cmd_avel);
      }
      break;
  }

  return 0;
}

uint8_t maneuv3r_rotateTracker_FSM = 0;// Tracking's state machine for rotating.
typedef struct maneuv3r_rotatetracker_t {
  float cmd_avel;
  float e_orient;// Orientation error (SV - PV)

  float Intg_e_orient;// Integral accumulator of the e_orient
  float Diff_e_orient;// Differentiator output
  float prev_e_orient;// z-1 of the e_orient
};
maneuv3r_rotatetracker_t rotatetracker_t;

// Rotate by a specific angle (relative to previous orientation)
uint8_t maneuv3r_rotateTracker(float rotate) {

  switch (maneuv3r_rotateTracker_FSM) {
    case 0:// Setup state
      {
        // Clear previous calculation
        rotatetracker_t.Intg_e_orient = 0.0;

        // Orientation setpoint
        rotate += robot_odom->pos_az;// calculate real setpoint
        maneuv3r_rotateTracker_FSM = 1;
      }
      break;
    case 1:// Tracking state
      {
        // Calculate the orientation error e_orient(z)
        rotatetracker_t.e_orient = rotate - robot_odom->pos_az;// pos_az is highly recommended to derive from either Gyro, Mag or fusion of both
  
        // PID controller algorithm for rotating
        rotatetracker_t.Intg_e_orient += rotatetracker_t.e_orient * pid_rotate_t.Ki;// I term
        rotatetracker_t.Diff_e_orient =
          (rotatetracker_t.e_orient - rotatetracker_t.prev_e_orient)
          * pid_rotate_t.Kd;                                                      // D term
        rotatetracker_t.prev_e_orient = rotatetracker_t.e_orient;                   // make a unit delay
  
        rotatetracker_t.cmd_avel =
          (rotatetracker_t.e_orient * pid_rotate_t.Kp) +
          rotatetracker_t.Intg_e_orient +
          rotatetracker_t.Diff_e_orient;

        // Angular velocity envelope
        // Max envelope
           rotatetracker_t.cmd_avel = 
              constrain(
                rotatetracker_t.cmd_avel,
                -pid_rotate_t.max_vang,
                pid_rotate_t.max_vang
              );
        // Dead band
        if((rotatetracker_t.cmd_avel < pid_rotate_t.min_vang) && (rotatetracker_t.cmd_avel > -pid_rotate_t.min_vang))
          rotatetracker_t.cmd_avel = 0.0;
  
        if (abs(rotatetracker_t.e_orient) < ROTATION_TOL) { // meets the orientation tolerance
          maneuv3r_update_Cmdvel(0.0, 0.0, 0.0);
          maneuv3r_rotateTracker_FSM = 0;
          return 1;
          break;
        }
  
        maneuv3r_update_Cmdvel(0.0, 0.0, rotatetracker_t.cmd_avel);
      }
      break;
  }

  return 0;
}

uint8_t maneuv3r_twizzlesTracker_FSM = 0;// Tracking's state machine for walk and rotate.
typedef struct maneuv3r_twizzlestracker_t {
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
  float initial_orient;
  float e_orient;// Orientation error (SV - PV)

  float Intg_e_orient;// Integral accumulator of the e_orient
  float Diff_e_orient;// Differentiator output
  float prev_e_orient;// z-1 of the e_orient
};
maneuv3r_twizzlestracker_t twizzlestracker_t;

// Allow robot to move in certain direction while also rotate to specific orientation.
// Borrowed the term from Ice Skating. Twizzles is "moving while also rotating".
// dist : moving distance
// heading : heading angle (moving direction)
// rotate : orientation at the destination (relative to previous orientation)
uint8_t maneuv3r_twizzlesTracker(float dist, float heading, float rotate) {

  switch (maneuv3r_twizzlesTracker_FSM) {
    case 0:// Setup state
      {
        // Clear previous calculations
        twizzlestracker_t.Intg_e_dist = 0.0;
        twizzlestracker_t.Intg_e_orient = 0.0;
        
        // Destination setpoint
        twizzlestracker_t.dest_x = (dist * cos(heading)) + robot_odom->pos_x;
        twizzlestracker_t.dest_y = (dist * sin(heading)) + robot_odom->pos_y;

        // Orientation setpoint
        twizzlestracker_t.initial_orient = rotate + robot_odom->pos_az;
        
        maneuv3r_twizzlesTracker_FSM = 1;
      }
      break;
      
    case 1:// Tracking state
      {
        /*============================== BEGIN ORIENTATION ================================*/
        // Calculate the orientation error e_orient(z)
        twizzlestracker_t.e_orient =  twizzlestracker_t.initial_orient - robot_odom->pos_az;// pos_az is highly recommended to derive from either Gyro, Mag or fusion of both
      
        // PID controller algorithm for rotating
        twizzlestracker_t.Intg_e_orient += twizzlestracker_t.e_orient * pid_twizzles_t.Ki;// I term
        twizzlestracker_t.Diff_e_orient =
          (twizzlestracker_t.e_orient - twizzlestracker_t.prev_e_orient)
          * pid_twizzles_t.Kd;                                                          // D term
        twizzlestracker_t.prev_e_orient = twizzlestracker_t.e_orient;                 // make a unit delay
      
        twizzlestracker_t.cmd_avel =
          (twizzlestracker_t.e_orient * pid_twizzles_t.Kp) +
          twizzlestracker_t.Intg_e_orient +
          twizzlestracker_t.Diff_e_orient;

        // Angular velocity envelope
        // Max envelope
           twizzlestracker_t.cmd_avel = 
              constrain(
                twizzlestracker_t.cmd_avel,
                -1.8, // 2.0 rad/s
                1.8
              );
        // Dead band
        if((twizzlestracker_t.cmd_avel < pid_rotate_t.min_vang) && (twizzlestracker_t.cmd_avel > -pid_rotate_t.min_vang))
          twizzlestracker_t.cmd_avel = 0.0;
        /*============================== END ORIENTATION ================================*/


        /*========================== BEGIN LINEAR VELOCITY===================================*/
        // Calculate the current distance (R) in polar coordinates
        twizzlestracker_t.diff_x = twizzlestracker_t.dest_x - robot_odom->pos_x;// SV - PV
        twizzlestracker_t.diff_y = twizzlestracker_t.dest_y - robot_odom->pos_y;// SV - PV

        /*============================ BEGIN VELOCITY HEADING ================================*/
        // Maintain heading to the destination
        twizzlestracker_t.cmd_heading = atan2pi(twizzlestracker_t.diff_y, twizzlestracker_t.diff_x);// Calculate the heading
        /*============================== END VELOCITY HEADING ================================*/
        
        // Take a square
        twizzlestracker_t.diff_x = twizzlestracker_t.diff_x * twizzlestracker_t.diff_x;// diff_x^2
        twizzlestracker_t.diff_y = twizzlestracker_t.diff_y * twizzlestracker_t.diff_y;// diff_y^2
        // Claculate distance error e_dist(z)
        // e_dist(z) = sqrt( (dest_x - pos_x)^2 + (dest_y - pos_y)^2 );
        twizzlestracker_t.e_dist = sqrt(twizzlestracker_t.diff_x + twizzlestracker_t.diff_y);// Euclidean distance
      
        // PID controller algorithm for distance tracking
        twizzlestracker_t.Intg_e_dist += twizzlestracker_t.e_dist * pid_walk_t.Ki;// I term
      
        twizzlestracker_t.Diff_e_dist =
          (twizzlestracker_t.e_dist - twizzlestracker_t.prev_e_dist)
          * pid_walk_t.Kd;                                                // D term
        twizzlestracker_t.prev_e_dist = twizzlestracker_t.e_dist;                 // make a unit delay
      
        // Finally calculate the lienar velocity CV (command value)
        twizzlestracker_t.cmd_lvel =
          (twizzlestracker_t.e_dist * pid_walk_t.Kp) +
          twizzlestracker_t.Intg_e_dist +
          twizzlestracker_t.Diff_e_dist;

        // Linear velocity envelope
        // Max envelope
        if(twizzlestracker_t.cmd_lvel > pid_walk_t.max_vlin)
          twizzlestracker_t.cmd_lvel = pid_walk_t.max_vlin;
        else if(twizzlestracker_t.cmd_lvel < -pid_walk_t.max_vlin)
          twizzlestracker_t.cmd_lvel = -pid_walk_t.max_vlin;
        // Dead band
        if((twizzlestracker_t.cmd_lvel < pid_walk_t.min_vlin) && (twizzlestracker_t.cmd_lvel > -pid_walk_t.min_vlin))
          twizzlestracker_t.cmd_lvel = 0.0;
        /*============================== END LINEAR VELOCITY  ================================*/

        // Goal checker
        
        // Check if rotation meets the tolerance 
        if(abs(twizzlestracker_t.e_orient) < ROTATION_TOL){
          twizzlestracker_t.cmd_avel = 0.0;
        }
        // Check if travel distance meets the tolerance
        if(abs(twizzlestracker_t.e_dist) < DISTANCE_TOL){
          twizzlestracker_t.cmd_lvel = 0.0;
        }

        // Once all goals were satisfied, stop the robot
        if ((abs(twizzlestracker_t.e_orient) < ROTATION_TOL) && (abs(twizzlestracker_t.e_dist) < DISTANCE_TOL)) {
          maneuv3r_update_Cmdvel(0.0, 0.0, 0.0);
          maneuv3r_twizzlesTracker_FSM = 0;
          return 1;
        }
        
        maneuv3r_update_Cmdvel(twizzlestracker_t.cmd_lvel, twizzlestracker_t.cmd_heading - robot_odom->pos_abs_az, twizzlestracker_t.cmd_avel);
      }
      break;
  }

  return 0;
}
