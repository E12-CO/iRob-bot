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
  float odom_y_weight
  ){
  target_odom = bot_odom;
  odom_vx_filter_t.complementary_weight = odom_x_weight;
  odom_vy_filter_t.complementary_weight = odom_y_weight;
  
}

void fuser_processYaw(){
  
  
  
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
