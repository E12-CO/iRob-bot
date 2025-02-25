// TODO : use fixed-point math for speed and resource.

#define DISTANCE_TOL 0.01f
#define ROTATION_TOL 0.043633f

pidConst_t pid_walk_t;// PID constant for linear motion
pidConst_t pid_twizzles_t;// PID constant for twizzles rotate control
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
  float Kd_twizzles) {

  pid_walk_t.Kp = Kp_walk;
  pid_walk_t.Ki = Ki_walk;
  pid_walk_t.Kd = Kd_walk;
  pid_walk_t.min_vlin = min_walk;
  pid_walk_t.max_vlin = max_walk;

  pid_twizzles_t.Kp =  Kp_twizzles;
  pid_twizzles_t.Ki =  Ki_twizzles;
  pid_twizzles_t.Kd =  Kd_twizzles;
}

void maneuv3r_update_Cmdvel(float vel, float heading, float az) {
  // Convert the R and Theta (polar coordinates) into X and Y velocity component (Cartesian coordinates).
  robot_cmd->vlx_out = vel * cos(heading);
  robot_cmd->vly_out = vel * sin(heading);

  // Commading angular velocity.
  robot_cmd->vaz_out = az;
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

float fVel;
bool vTargetReached = false;

// Allow robot to move in certain direction while also rotate to specific orientation.
// Borrowed the term from Ice Skating. Twizzles is "moving while also rotating".

// px and py : absolute x and y coordiate in the world
// orient : orientation at the destination (absolute to world)
uint8_t maneuv3r_twizzlesTrackerAbs(float px, float py, float orient){
   switch (maneuv3r_twizzlesTracker_FSM) {
    case 0:// Setup state
      {
        // Clear previous calculations
        twizzlestracker_t.Intg_e_dist = 0.0;
        twizzlestracker_t.Intg_e_orient = 0.0;
        
        // Destination setpoint
        twizzlestracker_t.dest_x = px;
        twizzlestracker_t.dest_y = py;

        // Orientation setpoint
        twizzlestracker_t.initial_orient = orient;

        // Reset velocity smoother
        vTargetReached = false;
        fVel = 0.0;
        
        maneuv3r_twizzlesTracker_FSM = 1;
      }
      break;
      
    case 1:// Tracking state
      {
        /*============================== BEGIN ORIENTATION ================================*/
        // Calculate the orientation error e_orient(z)
        twizzlestracker_t.e_orient =  twizzlestracker_t.initial_orient - robot_odom->pos_az;// pos_az is highly recommended to derive from either Gyro, Mag or fusion of both
        if(abs(twizzlestracker_t.e_orient) > 3.141593){
          if(twizzlestracker_t.e_orient < 0.0)
            twizzlestracker_t.e_orient += 6.283185;
          else
            twizzlestracker_t.e_orient -= 6.283185;
        }
        
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
                -1.8, // 1.5 rad/s
                1.8
              );
        // Dead band
        if((twizzlestracker_t.cmd_avel < 0.0005) && (twizzlestracker_t.cmd_avel > -0.0005))
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
          twizzlestracker_t.Intg_e_orient = 0.0;
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

        if(vTargetReached == false)
          fVel = ((1 - 0.08) * fVel) + (0.08 * twizzlestracker_t.cmd_lvel);
        else
          fVel = twizzlestracker_t.cmd_lvel;
          
        if(abs(abs(fVel) - abs(twizzlestracker_t.cmd_lvel)) < 0.05)
          vTargetReached = true;
        else if (abs(abs(fVel) - abs(twizzlestracker_t.cmd_lvel)) > 0.3)
           vTargetReached = false;
           
        maneuv3r_update_Cmdvel(fVel, twizzlestracker_t.cmd_heading - robot_odom->pos_abs_az, twizzlestracker_t.cmd_avel);
      }
      break;
  }

  return 0;
}

// dist : moving distance (relative to current position)
// heading : heading angle (moving direction relative to current position)
// orient : orientation at the destination (absolute to world)
uint8_t maneuv3r_twizzlesTracker(float dist, float heading, float orient) {
 return maneuv3r_twizzlesTrackerAbs(
        (dist * cos(heading)) + robot_odom->pos_x,
        (dist * sin(heading)) + robot_odom->pos_y,
        orient);
}

uint8_t maneuv3r_arcTracker_FSM = 0;// Tracking's state machine for arc walk.
maneuv3r_twizzlestracker_t arctracker_t;
