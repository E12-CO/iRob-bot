#include <Wire.h>
#include "stdodom.h"

// Robot dimension
#define OMNI_WHEEL_D  0.0727f // 72.7mm distance from robot center to wheel center
#define OMNI_WHEEL_R  0.0190f // 19.0mm wheel radius
#define OMNI_SINE_120 0.8660f // Sine(120 degree) in rad unit

// Conversion Constant
//#define RAD_S_TO_RPM 954.93 // rad/s to RPM with 1:100 gearbox scaling for commanding motor 
#define RAD_S_TO_RPM 318.31 // rad/s to RPM with 1:30 gearbox scaling for commanding motor

// Control system related
#define LOOP_TIME         8 // Loop time is 8ms -> 125Hz

// Odom mouse sensor ADNS5050
#define ADNS5050_SPI_NCS  5
#define ADNS5050_SPI_SIO  19
#define ADNS5050_SPI_SCK  18

// Robot odometry and cmd_vel
odometry_t omni_odom;// Robot odometry frame data
cmdvel_t omni_cmdvel;// Robot base-link frame velocity command
wheelvel_t omni_wheelvel;// Per-wheel velocity command

float atan2pi(float y, float x) {
  float at = atan2(y, x);
  if (at < 0.0)
    at += 6.28319;
  return at;
}

void sensor_Init() {
  /* Begin Sensor Initialization */
  //  adns5050_init(
  //    ADNS5050_SPI_NCS,
  //    ADNS5050_SPI_SIO,
  //    ADNS5050_SPI_SCK
  //  );
  //  // Probe for sensor
  //  if (adns5050_Probe()) {
  //    Serial.println("ADNS 5050 probing error!");
  //    while (1);
  //  }
  //  // Set CPI to 625 cpi
  //  adns5050_set625cpi();
  //  // turn LED on
  //  adns5050_setLED();
  //
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  motor_omniInit(&omni_wheelvel);// Pass the pointer to Wheel velocity command

  delay(50);
  sensor_Init();

  /* Begin maneuv3r algorithm */
  maneuv3r_init(&omni_odom);
  maneuv3r_pidInit(
    1.8, 0.005, 0.0,  // Walk kPID Kp 1.8 Ki 0.0 Kd 0.0
    0.05, 0.5,     // Walk min and max velocity (m/s)

    1.0, 0.0, 0.0,  // Heading kPID

    1.8, 0.002, 0.0,  // Rotate kPID
    0.001, 20.0      // Rotate min and max velocity (rad/s)
  );

  maneuv3r_update_Cmdvel(&omni_cmdvel, 0.0, 0.0, 0.0);

  /* End maneuv3r algorithm */

  // Init encoder
  encoder_Init();// Initialize ESP32 Encoder interrupt.
  delay(10);
}
unsigned long runner_millis = 0;

inline void computeOdom() {
  encoder_doVel();// Estimate Wheel RPM
  //hmc5883l_getPosZ(&omni_odom);// Update robot orientation (az) referenced to north, also update angular velocity.
  itg3205_getPosZ(&omni_odom);
  //adns5050_updateVel(&omni_odom);// Update Robot velocity (vx, vy) read by mouse sensor.
  odometry_wheelOdom(&omni_odom);
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

uint8_t main_fsm = 0;
unsigned long cycle_counter = 0;
float heading_counter;
inline void robot_runner() {

  switch (main_fsm) {
    case 0:
      {
        //Serial.println("running robot");
        main_fsm = 1;
      }
      break;

    case 1:
      {
        if (maneuv3r_twizzlesTracker(&omni_odom, &omni_cmdvel, 0.3, 0.0, 3.141593)) {
           main_fsm = 2;
        }
      }
      break;

    case 2:
      {
        if (maneuv3r_twizzlesTracker(&omni_odom, &omni_cmdvel, 0.3, 4.712391, 3.141593)) {
           main_fsm = 3;
        }
      }
      break;

    case 3:
      {
        if (maneuv3r_twizzlesTracker(&omni_odom, &omni_cmdvel, 0.3, 3.141593, 3.141593)) {
           main_fsm = 4;
        }
      }
      break;

    case 4:
      {
        if (maneuv3r_twizzlesTracker(&omni_odom, &omni_cmdvel, 0.3, 1.570797, 3.141593)) {
           main_fsm = 1;
        }
      }
      break;  
  }

}

void loop() {
  if ((micros() - runner_millis) > LOOP_TIME * 1000) {
    runner_millis = micros();
    computeOdom();
    robot_runner();
    cmd_vel(&omni_cmdvel);
    //    Serial.print(",V1:");
    //    Serial.print(omni_wheelvel.v1);
    //    Serial.print(",V2:");
    //    Serial.print(omni_wheelvel.v2);
    //    Serial.print(",V3:");
    //    Serial.print(omni_wheelvel.v3);
    //    Serial.print(",M1:");
    //    Serial.print(encoder_getM1());
    //    Serial.print(",M2:");
    //    Serial.print(encoder_getM2());
    //    Serial.print(",M3:");
    //    Serial.print(encoder_getM3());
    
  }
  //  hls_poll();// Will move to other core (plus network)
}
