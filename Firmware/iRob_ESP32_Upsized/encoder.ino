#define ENC_CONSTANT  267.8571f// 1 count * [(1 rev/28 count) * (1/0.008s) * (60s/1min)] = 267.8571 RPM

wheelvel_t *local_wheelfb_t;// Internal pointer used to access the Per-wheel velocity feedback.

ESP32Encoder enc_m1;
ESP32Encoder enc_m2;
ESP32Encoder enc_m3;
ESP32Encoder enc_m4;

// Motor LF
#define ENC_M1_A  39
#define ENC_M1_B  36
// Motor LB
#define ENC_M2_A  35
#define ENC_M2_B  34
// Motor RB
#define ENC_M3_A  27
#define ENC_M3_B  13
// Motor RF
#define ENC_M4_A  18
#define ENC_M4_B  17

typedef struct encoder_t{
  int32_t current_count;
  int32_t prev_count;
  float estimated_vel;
};

encoder_t enc_m1_t;
encoder_t enc_m2_t;
encoder_t enc_m3_t;
encoder_t enc_m4_t;

void encoder_Init(wheelvel_t *wheelfb_t){
  local_wheelfb_t = wheelfb_t;
  
  pinMode(ENC_M1_A, INPUT);
  pinMode(ENC_M1_B, INPUT);
  pinMode(ENC_M2_A, INPUT);
  pinMode(ENC_M2_B, INPUT);  
  pinMode(ENC_M3_A, INPUT);
  pinMode(ENC_M3_B, INPUT);
  pinMode(ENC_M4_A, INPUT);
  pinMode(ENC_M4_B, INPUT);

  enc_m1.attachFullQuad(ENC_M1_B, ENC_M1_A);
  enc_m2.attachFullQuad(ENC_M2_B, ENC_M2_A);
  enc_m3.attachFullQuad(ENC_M3_B, ENC_M3_A);
  enc_m4.attachFullQuad(ENC_M4_B, ENC_M4_A);
}

// robot configuration :
//                    +x
//        m1           ^
//       /  \          |
//      /    \  +y <---o
//    m3 ---- m2
//
// Estimate the velocity from count
void encoder_doVel(){
  enc_m1_t.current_count = enc_m1.getCount();
  enc_m2_t.current_count = enc_m2.getCount();
  enc_m3_t.current_count = enc_m3.getCount();
  enc_m4_t.current_count = enc_m4.getCount();

  enc_m1_t.estimated_vel += 
    (enc_m1_t.current_count - enc_m1_t.prev_count) * ENC_CONSTANT;
  enc_m1_t.estimated_vel = enc_m1_t.estimated_vel / 4;
  
  enc_m2_t.estimated_vel += 
    (enc_m2_t.current_count - enc_m2_t.prev_count) * ENC_CONSTANT;
  enc_m2_t.estimated_vel = enc_m2_t.estimated_vel / 4;
  
  enc_m3_t.estimated_vel += 
    (enc_m3_t.current_count - enc_m3_t.prev_count) * ENC_CONSTANT;
  enc_m3_t.estimated_vel = enc_m3_t.estimated_vel / 4;

  enc_m4_t.estimated_vel +=
    (enc_m4_t.current_count - enc_m4_t.prev_count) * ENC_CONSTANT;
  enc_m4_t.estimated_vel = enc_m4_t.estimated_vel / 4;

  local_wheelfb_t->v1 = enc_m1_t.estimated_vel;
  local_wheelfb_t->v2 = enc_m2_t.estimated_vel;
  local_wheelfb_t->v3 = enc_m3_t.estimated_vel;
  local_wheelfb_t->v4 = enc_m4_t.estimated_vel;

  enc_m1_t.prev_count = enc_m1_t.current_count;
  enc_m2_t.prev_count = enc_m2_t.current_count;
  enc_m3_t.prev_count = enc_m3_t.current_count;
  enc_m4_t.prev_count = enc_m4_t.current_count;
}

float encoder_getM1(){
  return enc_m1_t.estimated_vel;
}

float encoder_getM2(){
  return enc_m2_t.estimated_vel;
}

float encoder_getM3(){
  return enc_m3_t.estimated_vel;
}

float encoder_getM4(){
  return enc_m4_t.estimated_vel;  
}
