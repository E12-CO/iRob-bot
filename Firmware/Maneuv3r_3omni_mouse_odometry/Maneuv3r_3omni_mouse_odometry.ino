#include <Wire.h>
#include "stdodom.h"

// Conversion Constant

// Robot dimension
#define OMNI_WHEEL_D  0.0727f // 72.7mm distance from robot center to wheel center
#define OMNI_WHEEL_R  0.0190f // 19.0mm wheel radius

#define G1_48 // Select 1:48 gear ratio

#if defined(G1_100)
  #define RAD_S_TO_RPM 954.93f    // rad/s to RPM with 1:100 gearbox scaling for commanding motor 
  #define GEAR_RATIO  0.01f       // 1/100 gearbox ratio
#elif defined(G1_48)
  #define RAD_S_TO_RPM 458.3664f  // rad/s to RPM with 1:48 gearbox scaling for commanding motor 
  #define GEAR_RATIO  0.02083f    // 1/48 gearbox ratio
#elif defined(G1_30)
  #define RAD_S_TO_RPM 318.31f    // rad/s to RPM with 1:30 gearbox scaling for commanding motor
  #define GEAR_RATIO    0.0333f   // 1/30 gearbox ratio
#elif defined(G1_20)
  #define RAD_S_TO_RPM  190.9859  // rad/s to RPM with 1:20 gearbox scaling for commanding motor
  #define GEAR_RATIO  0.05f       // 1/20 gearbox ratio
#elif defined(G1_10)
  #define RAD_S_TO_RPM  95.493    // rad/s to RPM with 1:10 gearbox scaling for commanding motor
  #define GEAR_RATIO 0.10f        // 1/10 gearbox ratio
#else
  #error "Please select Gear ratio!"
#endif

// Math constants
#define RPM_TO_RAD_S  0.1047f // 1 rpm == 0.1047 rad/s 
#define OMNI_SINE_120 0.866025f // Sine(120 degree) in rad unit
#define F2_SQRT3      1.1547f // 2/sqrt(3)

// Control system related
#define LOOP_TIME         8 // Loop time is 8ms -> 125Hz

// Odom mouse sensor ADNS5050
#define ADNS5050_SPI_NCS  5
#define ADNS5050_SPI_SIO  19
#define ADNS5050_SPI_SCK  18
#define ADNS5050_NRST     23

// Global Typedef
// Robot odometry and cmd_vel
odometry_t omni_odom;// Robot odometry frame data
odometry_t optical_odom;// Optical odometry (only X and Y vel)
odometry_t wheel_odom;// Wheel odometry (Only X and Y vel)

cmdvel_t omni_cmdvel;// Robot base-link frame velocity command
wheelvel_t omni_wheelvel;// Per-wheel velocity command

TaskHandle_t Core0_t;

// Global variable
unsigned long runner_millis = 0;
unsigned long qcheck_millis = 0;
uint8_t main_fsm = 0;

float atan2pi(float y, float x) {
  float at = atan2(y, x);
  if (at < 0.0)
    at += 6.28319;
  return at;
}

void sensor_Init() {
  /* Begin Sensor Initialization */
  adns5050_init(
    ADNS5050_SPI_NCS,
    ADNS5050_SPI_SIO,
    ADNS5050_SPI_SCK,
    ADNS5050_NRST
  );
  // Probe for sensor
  if (adns5050_Probe()) {
    Serial.println("ADNS 5050 probing error!");
    while (1);
  }
  // Set CPI 
  adns5050_setcpi();
  // turn LED on
  adns5050_setLED();
  
  // Start I2C
  Wire.begin();

  // Init magnetometer
  //hmc5883l_init();

  // Init Gyro
  itg3205_init();

  // Init HLS_LFCD2 LiDAR
  hls_initLiDAR();
  /* end Sensor Initialization */
}

void task_Init(){
  
  xTaskCreatePinnedToCore(
      loop0, /* Function to implement the task */
      "network", /* Name of the task */
      40000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &Core0_t,  /* Task handle. */
      0); /* Core where the task should run */  
        
}

void setup() {
  // put your setup code here, to run once:
  motor_omniInit(&omni_wheelvel);// Pass the pointer to Wheel velocity command
  Serial.begin(115200);
  task_Init();

  pinMode(2, OUTPUT);
  pinMode(0, INPUT);
  while(digitalRead(0));
  Serial.println("Initializing...");
  digitalWrite(2, HIGH);
  // Init encoder
  encoder_Init();// Initialize ESP32 Encoder interrupt.
  delay(200);
  sensor_Init();
  digitalWrite(2, LOW);
  Serial.println("Done!");

  // Initialize fuser
  fuser_Init(
    &omni_odom,   // Robot's odometry
    0.625,        // Filter weight of the wheel odom (vel_x) 0.625
    0.625,        // Filter weight of the wheel odom (vel_y) 0.625
    1.0          // Filter weight of the gyro angular velocity (vel_az)
  );
  
  /* Begin maneuv3r algorithm */
  maneuv3r_init(
    &omni_odom, // Robot's system-wide odometry
    &omni_cmdvel // Robot's system-wide cmd_vel
    );
  maneuv3r_pidInit(
    1.2, 0.0005, 0.0,   // Walk kPID Kp 1.8 Ki 0.0005 Kd 0.0
    0.005, 0.26,        // Walk min and max velocity (m/s)

    0.0, 0.3, 0.0045,  // Twizzle rotate kPID Kp 0.86 Ki 0.00085 Kd 0.003

    1.4, 0.0015, 0.0,  // Rotate kPID
    0.0005, 3.5    // Rotate min and max velocity (4.0 rad/s)
  );

  maneuv3r_update_Cmdvel(0.0, 0.0, 0.0);
  /* End maneuv3r algorithm */
}

inline void computeOdom() {
  encoder_doVel();// Estimate Wheel RPM.

  // Re-using the optical and wheel odometry struct.
  itg3205_getPosZ(&optical_odom);       // Integrate the angular velocity to estimate orientation.
  //hmc5883l_getPosZ(&wheel_odom);        // Update robot orientation (az) referenced to north, also update angular velocity.
  fuser_processYaw(&optical_odom, &wheel_odom);// Fuse Gyro angular vel with Mag angular vel.
  
  adns5050_updateVel(&optical_odom);// Get X and Y velocity from mouse sensor.
  odometry_wheelOdom(&wheel_odom);  // Get X and Y velocity from wheel encoders.
  fuser_processOdom(&optical_odom, &wheel_odom); // Fuse Optical flow odom with Wheel odom.

  odometry_posUpdate(&omni_odom);// Update robot X, Y position by integrating vx and vy.
}


// robot configuration :
//                    +x
//        v1           ^
//       /  \          |
//      /    \  +y <---o
//    v3 ---- v2
// Inverse kinematic below :
inline void cmd_vel(cmdvel_t *command_vel) {
  omni_wheelvel.v1 =
    (command_vel->vly_out +
     (command_vel->vaz_out * OMNI_WHEEL_D)
    ) / OMNI_WHEEL_R;

  omni_wheelvel.v2 =
    ((command_vel->vlx_out * OMNI_SINE_120) +
     (command_vel->vly_out * -0.5) +
     (command_vel->vaz_out * OMNI_WHEEL_D)
    ) / OMNI_WHEEL_R;

  omni_wheelvel.v3 =
    ((command_vel->vlx_out * -OMNI_SINE_120) +
     (command_vel->vly_out * -0.5) +
     (command_vel->vaz_out * OMNI_WHEEL_D)
    ) / OMNI_WHEEL_R;

  omni_wheelvel.v1 *= RAD_S_TO_RPM;
  omni_wheelvel.v2 *= RAD_S_TO_RPM;
  omni_wheelvel.v3 *= RAD_S_TO_RPM;

  motor_pidUpdate();// Do motor PID algorithm
}
uint8_t play_cnt = 0;
void robot_runner() {

  switch (main_fsm) {
    case 0:
      {
//        if(maneuv3r_walkTracker(0.8, -1.570797))
          main_fsm = 1;
      }
      break;

    case 1:
      {
        if (maneuv3r_twizzlesTracker(0.8, 0.0, 0.0)) {
           main_fsm = 2;
        }
      }
      break;

    case 2:
      {
        if (maneuv3r_twizzlesTracker(0.8, -1.570797, 0.0)) {
           main_fsm = 3;
        }
      }
      break;

    case 3:
      {
        if (maneuv3r_twizzlesTracker(0.8, -3.141593, 0.0)) {
           main_fsm = 4;
        }
      }
      break;

    case 4:
      {
        if (maneuv3r_twizzlesTracker(0.8, 1.570797, 0.0)) {
            play_cnt++;
            if(play_cnt == 4)
              main_fsm = 255;
            else
              main_fsm = 1;
        }
      }
      break;  

    case 255:
      {

      }
      break;
  }

}
void loop() {
  //Control loop here
  if ((micros() - runner_millis) > LOOP_TIME * 1000) {
    runner_millis = micros();
    computeOdom();
    robot_runner();
    cmd_vel(&omni_cmdvel);
  }
}

void loop0(void *pvParameters){
  
  while(1){
//      Serial.print(",Surface Q:");
//      Serial.print(adns5050_getSurfaceQ());
      Serial.print(",Vx :");
      Serial.print(omni_odom.vel_x);
      Serial.print(",Vy :");
      Serial.print(omni_odom.vel_y);
      Serial.print(",Pos X:");
      Serial.print(omni_odom.pos_x);
      Serial.print(",Pos Y:");
      Serial.print(omni_odom.pos_y);
      Serial.print(",Orient:");
      Serial.print(omni_odom.pos_abs_az);
      Serial.print(",Heading");
      Serial.print(omni_odom.pos_heading);
//      Serial.print(",V1:");
//      Serial.print(omni_wheelvel.v1);
//      Serial.print(",V2:");
//      Serial.print(omni_wheelvel.v2);
//      Serial.print(",V3:");
//      Serial.print(omni_wheelvel.v3);
//      Serial.print(",M1:");
//      Serial.print(encoder_getM1());
//      Serial.print(",M2:");
//      Serial.print(encoder_getM2());
//      Serial.print(",M3:");
//      Serial.print(encoder_getM3());
      Serial.println();
      delay(100);
      //  hls_poll();
  }
}
