#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include <stm32f303xc.h>

#include "clk.h"
#include "systick_millis.h"

#include "app_gpio.h"

#include "app_ros_comm.h"
#include "app_io_manager.h"
#include "app_motor_control.h"

#define LOOP_TIME_MOTORCTRL		(1 * 10)
#define LOOP_TIME_SENSOR			(20 * 10)

void app_robot_appInit();
void app_robot_runner();

#endif