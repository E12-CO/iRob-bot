#include <ESP32Encoder.h>

#define ENC_CONSTANT  267.8571f// 1 count * [(1 rev/28 count) * (1/0.008s) * (60s/1min)] = 267.8571 RPM

#define ALPHA_CONST   0.8f
#define BETA_CONST    0.0f

ESP32Encoder enc_m1;
ESP32Encoder enc_m2;
ESP32Encoder enc_m3;

#define ENC_M1_A  33
#define ENC_M1_B  32

#define ENC_M2_A  35
#define ENC_M2_B  34

#define ENC_M3_A  39
#define ENC_M3_B  36

typedef struct encoder_t{
  int32_t current_count;
  int32_t prev_count;
  float estimated_vel;
};

encoder_t enc_m1_t;
encoder_t enc_m2_t;
encoder_t enc_m3_t;

void encoder_Init(){
  pinMode(ENC_M1_A, INPUT);
  pinMode(ENC_M1_B, INPUT);
  pinMode(ENC_M2_A, INPUT);
  pinMode(ENC_M2_B, INPUT);  
  pinMode(ENC_M3_A, INPUT);
  pinMode(ENC_M3_B, INPUT);

  enc_m1.attachFullQuad(ENC_M1_B, ENC_M1_A);
  enc_m2.attachFullQuad(ENC_M2_B, ENC_M2_A);
  enc_m3.attachFullQuad(ENC_M3_B, ENC_M3_A);
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

  enc_m1_t.estimated_vel += 
    (enc_m1_t.current_count - enc_m1_t.prev_count) * ENC_CONSTANT;
  enc_m1_t.estimated_vel = enc_m1_t.estimated_vel / 4;
  
  enc_m2_t.estimated_vel += 
    (enc_m2_t.current_count - enc_m2_t.prev_count) * ENC_CONSTANT;
  enc_m2_t.estimated_vel = enc_m2_t.estimated_vel / 4;
  
  enc_m3_t.estimated_vel += 
    (enc_m3_t.current_count - enc_m3_t.prev_count) * ENC_CONSTANT;
  enc_m3_t.estimated_vel = enc_m3_t.estimated_vel / 4;

  enc_m1_t.prev_count = enc_m1_t.current_count;
  enc_m2_t.prev_count = enc_m2_t.current_count;
  enc_m3_t.prev_count = enc_m3_t.current_count;
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

// Experimenting with the Encoder Tracking filter from
// the "Teaching Old Motors New Tricks -- Part 4"
// by Dave Wilson
int32_t current_count;
int32_t delta_count;

float alpha_term, beta_term;
float intg_alpha, intg_beta;

float estimated_velocity;
int32_t estimated_count;
float encoder_observerRunner(){
  current_count = enc_m1.getCount();
  // Calculate the position error from the measured count 
  // vs the observer
  delta_count = current_count - estimated_count;

  // Calculate the after-gain value 
  alpha_term = ALPHA_CONST * delta_count;
  beta_term = BETA_CONST * delta_count;// This term is Acceleration

  // Integrate beta term 
  intg_beta += beta_term;
  estimated_velocity = intg_beta + alpha_term;// This term is Velocity
  
  estimated_count = estimated_velocity + intg_alpha;// This term is Position

  // that also used as the feedback to check position error
  intg_alpha = estimated_count;

  return estimated_velocity;
}
