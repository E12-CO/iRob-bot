wheelvel_t *local_wheelvel_t;// Internal pointer used to access the Per-wheel velocity command.

#define MAX_SPEED  4095 // Max pwm 
#define MIN_SPEED  100  // Min pwm
//#define INTG_CAP   // Cap the integrator term

#define M1_H  25
#define M1_L  26

#define M2_H  27
#define M2_L  14

#define M3_H  13
#define M3_L  4

// TODO : Retune to get better input response.
motor_var_t omni_m1_t = {
  .motor_Kp = 4.9,// Motor 1 Kp
  .motor_Ki = 0.14,// Motor 1 Ki

  .curr_speed = 0.0,
  .prev_speed = 0.0,
  .e_speed = 0.0,
  .Intg_e_speed = 0.0,
  .speed_cmd = 0.0
};

motor_var_t omni_m2_t = {
  .motor_Kp = 4.0,// Motor 2 Kp
  .motor_Ki = 0.14,// Motor 2 Ki

  .curr_speed = 0.0,
  .prev_speed = 0.0,
  .e_speed = 0.0,
  .Intg_e_speed = 0.0,
  .speed_cmd = 0.0
};

motor_var_t omni_m3_t = {
  .motor_Kp = 4.0,// Motor 3 Kp
  .motor_Ki = 0.14,// Motor 3 Ki

  .curr_speed = 0.0,
  .prev_speed = 0.0,
  .e_speed = 0.0,
  .Intg_e_speed = 0.0,
  .speed_cmd = 0.0
};

void motor_omniInit(wheelvel_t *omni_wheelvel) {
  local_wheelvel_t = omni_wheelvel;

  pinMode(M1_H, OUTPUT);
  pinMode(M1_L, OUTPUT);
  pinMode(M2_H, OUTPUT);
  pinMode(M2_L, OUTPUT);
  pinMode(M3_H, OUTPUT);
  pinMode(M3_L, OUTPUT);

  ledcAttachChannel(M1_H, 12000, 12, 0);
  ledcAttachChannel(M1_L, 12000, 12, 1);
  ledcAttachChannel(M2_H, 12000, 12, 2);
  ledcAttachChannel(M2_L, 12000, 12, 3);
  ledcAttachChannel(M3_H, 12000, 12, 4);
  ledcAttachChannel(M3_L, 12000, 12, 5);

  motor_out(0, 0, 0);
}


// robot configuration :
//                    +x
//        m1           ^
//       /  \          |
//      /    \  +y <---o
//    m3 ---- m2
//
// Spinning direction (viewing from the fron of the wheel)
// CW -> command positive
// CCW -> command negative
float error_percent;
void motor_doPID(motor_var_t *motor_ptr_t, float sp_speed) {
  if(motor_ptr_t->prev_speed != sp_speed)
    motor_ptr_t->Intg_e_speed = 0.0;

  motor_ptr_t->prev_speed = sp_speed;
  motor_ptr_t->e_speed = sp_speed - motor_ptr_t->curr_speed;

  error_percent = motor_ptr_t->e_speed / sp_speed;

  if(error_percent < 0.90)
    motor_ptr_t->Intg_e_speed += motor_ptr_t->e_speed * motor_ptr_t->motor_Ki;
    
  motor_ptr_t->speed_cmd =
    round(
      (motor_ptr_t->e_speed * motor_ptr_t->motor_Kp) +
      motor_ptr_t->Intg_e_speed
    );

  // Speed cap
  if (motor_ptr_t->speed_cmd > MAX_SPEED)
    motor_ptr_t->speed_cmd = MAX_SPEED;
  else if (motor_ptr_t->speed_cmd < -MAX_SPEED)
    motor_ptr_t->speed_cmd = -MAX_SPEED;

//  if((motor_ptr_t->speed_cmd < MIN_SPEED) && (motor_ptr_t->speed_cmd > -MIN_SPEED))  
//    motor_ptr_t->speed_cmd = 0;
}

void motor_pidUpdate() {
  omni_m1_t.curr_speed = encoder_getM1();
  omni_m2_t.curr_speed = encoder_getM2();
  omni_m3_t.curr_speed = encoder_getM3();

  motor_doPID(&omni_m1_t, local_wheelvel_t->v1);
  motor_doPID(&omni_m2_t, local_wheelvel_t->v2);
  motor_doPID(&omni_m3_t, local_wheelvel_t->v3);

  motor_out(
    omni_m1_t.speed_cmd,
    omni_m2_t.speed_cmd,
    omni_m3_t.speed_cmd
    );
}

void motor_out(int m1, int m2, int m3) {

  ledcWrite(M1_H, m1 > 0 ? m1 : 0);
  ledcWrite(M1_L, m1 >= 0 ? 0 : -m1);

  ledcWrite(M2_H, m2 > 0 ? m2 : 0);
  ledcWrite(M2_L, m2 >= 0 ? 0 : -m2);

  ledcWrite(M3_H, m3 > 0 ? m3 : 0);
  ledcWrite(M3_L, m3 >= 0 ? 0 : -m3);
  
}
