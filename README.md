# iRob-bot

iRob-bot is a robot testing platform for Arduino and ROS2.
The goal is to have a autonomous robot with good odometry accuracy while having a simple motion control system that capable of manuvering complex movement.  

This robot is a part of R&D project at Robot club engineering KMITL (RB26).

# Why iRob-bot ?

Our club has a long experience with Mechanical and Electronics design. We have been doing these kind of works for almost three decades. But when it comes to autonomous robot. We're very new to this and still lacking a lot of fundamental skill such as motion control and planning. At the competition, our manual control robot worked very hard to gain a good amount of score, while our autonomous robot struggled to do a mission, let alone walking straight. 

Back in december 2023, there was a competition called ```CRU Robot Games```. 
This competition which each team has to compete using one manual control robot 
and one autonomous robot. In fact, we also participated in this same competition 
but without much success as we are very new to autonomous robot. (I personally participated back in 2022, That also my first time working with closed-loop 
control of DC motor with the autonomous robot, but again without much success).

There was a 3 wheeled omni robot from a team ```iRap```(robotics club from KMUTNB). I was intrigued by their autonomous robot, especially the motion control. Somehow it can do advanced motion like translational motion while also rotates 
around its own axis, or following the curve arc while also rotates around.

At the time I saw their robot in-action. I've had some previous experience from joining the club and working with holonomic robot (using mecanum/swiss wheel) . 
But nothing of us every came close to theirs. And we are still using ```robot oriented``` control. At best our robot did equiped with encoder and PI speed controlled for motor. But the robot still lack of a good motion
control system.

Tragidy strikes. Our club got critizied with a surprise budget cut by half. The reason was that we ```don't have a real work``` a.k.a we never win any of these competition despite a lots of work behind and knowledge that we gained and poured into each competition.

In out defense, we decided to start a R&D project to research and test out around the topic of robotics. The goal is to gain knowledge as much as possible to be used in competition. But also to prove them that every cent that we spend is turned into know-how that we then pass down to the next club member generation. 

This year is my last year at university as I will (hopefully) be graduated soon. I want to do something to help our junior members as a payback to what this club had taught me from the past 2 years. So I set out to learn and implement the motion control system. 

This is a good oppurtunity for me to revisit what I've seen back in 2023. I finally came up with a robot test platform called ```iRob-bot```, basically I "rob" the idea of using 3 wheeled omni robot from iRap, hence the name iRob-bot. along with the motion controller called ```Maneuv3r``` that will be on arduino and ROS2. 

# General Information

- Dimension : ⌀ : 145.4mm, Height included LiDAR : ~ 130mm
- Weight : ~0.4 kg
- Drivetrain ```omnidirectional 3 wheels (omni-wheel)```
- Motor ```N20 motor with 1:48 gearbox and 7PPR AB quadrature encoder```
- Drive ```DRV8835 on custom breakout board(sees Electronics folder)```
- Main controller ```ESP32 WROOM32```
- Odometry sensors
    - ```Wheel encoder``` (Moving average filtering)    
    - ```ADNS-5050``` optical flow sensor 
	- Wheel odometry and Optical Flow odometry fusion using ```complementary filter```
- IMU via I2C (9 axis GY-85 module)
    - ```ADXL345``` 3 axis Accelerometer (currently unused until I can implement EKF)
    - ```ITG3205``` 3 axis Gyro sensor
    - ```HMC5883L``` 3 axis Mag sensor (currently unused until I can implement EKF)
- LiDAR
    - ```HLS-LF2CD``` LiDAR (currently unused until I implemented ROS2 node)
- Digital Control running at ```125Hz```
    - PI speed control (No torque control)
    - PID position and orientation control
- ESP32 Dual core
    - Core 0 is responsible for Networking task and LiDAR reading (Work in progress)
    - Core 1 is working on Low-level control system

# Folders
- Documents : Schematic and diagram PDFs and more soon.
- Electronics : Electronics design file (EAGLE cad and Fusion 360 format)
- Firmware : Arduino firmware
- Hardware : CAD design of the robot in Fusion 360 archive
- Software : ROS2 node (Coming soon)