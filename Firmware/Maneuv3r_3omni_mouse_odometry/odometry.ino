// Estimate XY pose of the robot
void odometry_posUpdate(odometry_t *robot_odom) {
  float sinAz, cosAz;
  sinAz = sin(robot_odom->pos_az);
  cosAz = cos(robot_odom->pos_az);

  // Integrate the velocity to estimate postion
  robot_odom->pos_x +=
    (
      (robot_odom->vel_x * cosAz) -
      (robot_odom->vel_y * sinAz)
    ) * 0.008;
  robot_odom->pos_y +=
    (
      (robot_odom->vel_x * sinAz) +
      (robot_odom->vel_y * cosAz)
    ) * 0.008;

  // Calculate the measured velocity heading
  robot_odom->pos_heading = atan2pi(robot_odom->vel_y, robot_odom->vel_x);
}

//define GEAR_RATION  0.01f   // 1/100 gearbox ratio
#define GEAR_RATIO    0.0333f // 1/30 gearbox ratio
#define RPM_TO_RAD_S  0.1047f // 1 rpm == 0.1047 rad/s 
#define SQRT3_2       OMNI_SINE_120 // sqrt(3)/2
// robot configuration :
//                    +x
//        v1           ^
//       /  \          |
//      /    \  +y <---o
//    v3 ---- v2
// Forward kinematic below :
void odometry_wheelOdom(odometry_t *robot_odom){
  float v1, v2, v3;
  v1 = encoder_getM1() * RPM_TO_RAD_S *GEAR_RATIO;
  v2 = encoder_getM2() * RPM_TO_RAD_S *GEAR_RATIO;
  v3 = encoder_getM3() * RPM_TO_RAD_S *GEAR_RATIO;

  robot_odom->vel_x = (v2-v3) * SQRT3_2 * OMNI_WHEEL_R / 3;
  robot_odom->vel_y = (v1 - ((v2+v3) * 0.5)) * OMNI_WHEEL_R / 3;
}
