# iRob-bot

iRob-bot is a robot testing platform for Arduino and ROS2.  
This robot is a part of R&D project at Robot club engineering KMITL (RB26).

# General Information

- Dimension : To be measured
- Weight : To be measured
- Drivetrain ```omnidirectional 3 wheels (omni-wheel)```
- Motor ```N20 motor with 1:30* gearbox and 7PPR AB quadrature encoder```
- Drive ```DRV8835 on custom breakout board(sees Electronics folder)```
- Main controller ```ESP32 WROOM32```
- Odometry sensors
    - ```Wheel encoder``` (Moving average filtering)    
    - ```ADNS-5050``` optical flow sensor (W.I.P)
- IMU via I2C (9 axis GY-85 module)
    - ```ADXL345``` 3 axis Accelerometer
    - ```ITG3205``` 3 axis Gyro sensor
    - ```HMC5883L``` 3 axis Mag sensor
- LiDAR
    - ```HLS-LF2CD``` LiDAR
- Digital Control running at ```125Hz```
    - PI speed control (No torque control)
    - PID position and orientation control

*gearbox ration is yet to be confirmed.

# Folders
- Electrnics : Electronics design file (EAGLE cad format)
- Firmware : Arduino firmware
- Hardware : CAD design of the robot
- Software : ROS2 node