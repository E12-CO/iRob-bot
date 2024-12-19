wheelvel_t *local_wheelvel_t;// Internal pointer used to access the Per-wheel velocity command.

#define MAX_SPEED  4095 // Max pwm 
#define MIN_SPEED  5  // Min pwm

// Motor LF
#define M1_H  33
#define M1_L  32
// Motor LB
#define M2_H  26
#define M2_L  25
// Motor RB
#define M3_H  16
#define M3_L  4
// Motor RF
#define M4_H  21
#define M4_L  19

// TODO : Retune to get better input response.
motor_var_t mec_m1_t = {
  .motor_Kp = 1.2,// Motor 1 Kp
  .motor_Ki = 0.05,// Motor 1 Ki

  .curr_speed = 0.0,
  .prev_speed = 0.0,
  .e_speed = 0.0,
  .Intg_e_speed = 0.0,
  .speed_cmd = 0.0
};

motor_var_t mec_m2_t = {
  .motor_Kp = 1.2,// Motor 2 Kp
  .motor_Ki = 0.05,// Motor 2 Ki

  .curr_speed = 0.0,
  .prev_speed = 0.0,
  .e_speed = 0.0,
  .Intg_e_speed = 0.0,
  .speed_cmd = 0.0
};

motor_var_t mec_m3_t = {
  .motor_Kp = 1.2,// Motor 3 Kp
  .motor_Ki = 0.05,// Motor 3 Ki

  .curr_speed = 0.0,
  .prev_speed = 0.0,
  .e_speed = 0.0,
  .Intg_e_speed = 0.0,
  .speed_cmd = 0.0
};

motor_var_t mec_m4_t = {
  .motor_Kp = 1.2,// Motor 4 Kp
  .motor_Ki = 0.05,// Motor 4 Ki

  .curr_speed = 0.0,
  .prev_speed = 0.0,
  .e_speed = 0.0,
  .Intg_e_speed = 0.0,
  .speed_cmd = 0.0
};

void motor_Init(wheelvel_t *mec_wheelvel) {
  local_wheelvel_t = mec_wheelvel;

  pinMode(M1_H, OUTPUT);
  pinMode(M1_L, OUTPUT);
  pinMode(M2_H, OUTPUT);
  pinMode(M2_L, OUTPUT);
  pinMode(M3_H, OUTPUT);
  pinMode(M3_L, OUTPUT);
  pinMode(M4_H, OUTPUT);
  pinMode(M4_L, OUTPUT);

  ledcAttachChannel(M1_H, 12000, 12, 0);
  ledcAttachChannel(M1_L, 12000, 12, 1);
  ledcAttachChannel(M2_H, 12000, 12, 2);
  ledcAttachChannel(M2_L, 12000, 12, 3);
  ledcAttachChannel(M3_H, 12000, 12, 4);
  ledcAttachChannel(M3_L, 12000, 12, 5);
  ledcAttachChannel(M4_H, 12000, 12, 6);
  ledcAttachChannel(M4_L, 12000, 12, 7);

  motor_out(0, 0, 0, 0);
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
void motor_doPID(motor_var_t *motor_ptr_t, float sp_speed) {
  if(motor_ptr_t->prev_speed != sp_speed)
    motor_ptr_t->Intg_e_speed = 0.0;

  motor_ptr_t->prev_speed = sp_speed;
  motor_ptr_t->e_speed = sp_speed - motor_ptr_t->curr_speed;

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
}

void motor_pidUpdate() {
  mec_m1_t.curr_speed = encoder_getM1();// LF
  mec_m2_t.curr_speed = encoder_getM2();// LB
  mec_m3_t.curr_speed = encoder_getM3();// RB
  mec_m4_t.curr_speed = encoder_getM4();// RF
  
  motor_doPID(&mec_m1_t, local_wheelvel_t->v1);// LF
  motor_doPID(&mec_m2_t, local_wheelvel_t->v2);// LB
  motor_doPID(&mec_m3_t, local_wheelvel_t->v3);// RB
  motor_doPID(&mec_m4_t, local_wheelvel_t->v4);// RF
  
  motor_out(
    mec_m1_t.speed_cmd,
    mec_m2_t.speed_cmd,
    mec_m3_t.speed_cmd,
    mec_m4_t.speed_cmd
    );
}

void motor_out(int m1, int m2, int m3, int m4) {

  ledcWrite(M1_H, m1 > 0 ? m1 : 0);
  ledcWrite(M1_L, m1 >= 0 ? 0 : -m1);

  ledcWrite(M2_H, m2 > 0 ? m2 : 0);
  ledcWrite(M2_L, m2 >= 0 ? 0 : -m2);

  ledcWrite(M3_H, m3 > 0 ? m3 : 0);
  ledcWrite(M3_L, m3 >= 0 ? 0 : -m3);

  ledcWrite(M4_H, m4 > 0 ? m4 : 0);
  ledcWrite(M4_L, m4 >= 0 ? 0 : -m4);
}

void motor_eStop(){
  mec_m1_t.Intg_e_speed = 0.0f;
  mec_m2_t.Intg_e_speed = 0.0f;
  mec_m3_t.Intg_e_speed = 0.0f;
  mec_m4_t.Intg_e_speed = 0.0f;
}
