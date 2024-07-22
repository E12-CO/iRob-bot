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

// TODO : Fuse wheel odometry with the optical flow sensor.
// robot configuration :
//                    +x
//        v1           ^
//       /  \          |
//      /    \  +y <---o
//    v3 ---- v2
// Forward kinematic below :
void odometry_wheelOdom(odometry_t *wheel_odom){
  float v1, v2, v3;
  v1 = encoder_getM1() * RPM_TO_RAD_S * GEAR_RATIO;
  v2 = encoder_getM2() * RPM_TO_RAD_S * GEAR_RATIO;
  v3 = encoder_getM3() * RPM_TO_RAD_S * GEAR_RATIO;

  wheel_odom->vel_x = (v2-v3) * OMNI_SINE_120 * OMNI_WHEEL_R;
  wheel_odom->vel_y = (v1 - ((v2+v3) * 0.5)) * OMNI_WHEEL_R;
}
