// Short term -> wheel odom
// Long term -> Optical flow

// Short term -> Gyro sensor
// Long term -> Mag sensor


typedef struct{
  float complementary_weight;// Between 0.0 to 1.0
  float data_sum;
}fuser_param_t;

fuser_param_t yaw_filter_t;
fuser_param_t odom_vx_filter_t;
fuser_param_t odom_vy_filter_t;

odometry_t *target_odom;

void fuser_Init(
  odometry_t *bot_odom,
  float odom_x_weight,
  float odom_y_weight,
  float yaw_weight
  ){
  target_odom = bot_odom;
  odom_vx_filter_t.complementary_weight = odom_x_weight;
  odom_vy_filter_t.complementary_weight = odom_y_weight;
  yaw_filter_t.complementary_weight = yaw_weight;
}

void fuser_processYaw(odometry_t *gyro_odom, odometry_t *mag_odom){
  yaw_filter_t.data_sum = 
    (yaw_filter_t.complementary_weight * gyro_odom->vel_az)
    +
    ((1 - yaw_filter_t.complementary_weight) * mag_odom->vel_az);
  
  target_odom->vel_az = yaw_filter_t.data_sum;
  target_odom->pos_az += target_odom->vel_az * 0.008;
  target_odom->pos_abs_az += target_odom->vel_az * 0.008;
  if(target_odom->pos_abs_az > 6.28319)
    target_odom->pos_abs_az -= 6.28319;
  if(target_odom->pos_abs_az < 0.0)
    target_odom->pos_abs_az += 6.28319; 
}

void fuser_processOdom(odometry_t *optical_odom, odometry_t *wheel_odom){
  odom_vx_filter_t.data_sum = 
    (odom_vx_filter_t.complementary_weight * wheel_odom->vel_x)
    +
    ((1 - odom_vx_filter_t.complementary_weight) * optical_odom->vel_x);

  odom_vy_filter_t.data_sum = 
    (odom_vy_filter_t.complementary_weight * wheel_odom->vel_y)
    +
    ((1 - odom_vy_filter_t.complementary_weight) * optical_odom->vel_y);

  target_odom->vel_x = odom_vx_filter_t.data_sum;
  target_odom->vel_y = odom_vy_filter_t.data_sum;
}
