#include <Wire.h>
#include <ESP32Encoder.h>
#include "stdirob.h"

#define LOOP_TIME_MS    (8-1) // 125Hz - 8ms
#define SENSOR_TIME_MS  (20-1) // 50Hz - 20ms
#define COMM_TIMEOUT_MS (40-1) // 40ms communication timeout

// Private Pointers

// Private variables
unsigned long motor_millis = 0;
unsigned long led_millis = 0;
unsigned long commTimeout_millis = 0;

uint8_t led_fsm = 0;
uint8_t led_blinker = 0;

// Private typedefs
wheelvel_t wheevel_cmd_t; // Wheel velocity command
wheelvel_t wheelvel_fb_t; // Wheel velocity feedback

sensor3dRaw_t gyroData_t;
sensor3dRaw_t accelData_t;
sensor3dRaw_t magData_t;

TaskHandle_t Core0_t;

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
  irob_init();
  task_Init();
  irob_runner();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void loop0(void *pvParameters){
  motor_Init(&wheevel_cmd_t);
  encoder_Init(&wheelvel_fb_t);
  
  while(1){
    if((millis() - motor_millis) > LOOP_TIME_MS){
      motor_millis = millis();
      encoder_doVel();
      motor_pidUpdate();
    }    

    if(app_ros_comm_TxDone()){
      mpu_getData(
        &gyroData_t,
        &accelData_t
      );
    }
  }
  
}

void irob_init(){
  pinMode(2, OUTPUT);
  mpu_init();
  delay(10);
  mpu_gyroCalRoutine(&gyroData_t);
  app_ros_comm_init(
    &gyroData_t,
    &accelData_t,
    &magData_t,
    &wheevel_cmd_t,
    &wheelvel_fb_t
    );
    
}

void irob_runner(){
  while(1){
    app_ros_comm_runner();    
  }
}
