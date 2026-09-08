# Driving iRob with `teleop_twist_keyboard`

This guide is for anyone using the iRob ROS 2 workspace with the four-wheel
omni ESP32 robot. It assumes you are already in a terminal where ROS 2 is
available and the `ROS_iROB_V0` workspace overlay has been sourced.

The control path is:

```text
teleop_twist_keyboard -- /cmd_vel --> iRob_controller
    -- /irob_motor_cmd --> iRob_interface -- /dev/ESP32 --> ESP32
```

The current controller configuration uses the `Omni4` kinematic plugin. You
do not need to publish motor commands yourself; the controller converts the
keyboard velocity commands into four motor commands.

## Before driving

1. Turn off or lift the robot so the wheels cannot move unexpectedly.
2. Upload the ESP32 sketch in this directory. The firmware serial rate is
   `115200` baud.
3. Connect the ESP32 by USB. Ensure `/dev/ESP32` exists and points to the
   connected ESP32:

   ```bash
   ls -l /dev/ESP32
   ```

   If it does not exist, install the udev rule once:

   ```bash
   sudo cp src/iRob_bot_ros2/irob_controller/esp32.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules
   ```

   Unplug and reconnect the ESP32 after installing the rule.
4. Build the workspace after any source-code or parameter-file change, then
   source its overlay:

   ```bash
   colcon build --packages-select irob_interface irob_controller irob_launcher --symlink-install
   source install/setup.bash
   ```

## Start the robot nodes

Open three terminals after completing the setup above. ROS commands in each
terminal must use the same ROS domain and workspace overlay.

### Terminal 1: ESP32 interface

This node sends motor commands to the ESP32 and publishes motor feedback.

```bash
ros2 launch irob_launcher irob_interface_esp32.launch.py
```

### Terminal 2: kinematic controller

This node receives `/cmd_vel` and calculates the four wheel speeds.

```bash
ros2 launch irob_launcher irob_controller.launch.py
```

### Terminal 3: keyboard teleoperation

Install the keyboard package if needed:

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
```

Start at a low speed:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5
```

Keep this terminal focused while driving. `teleop_twist_keyboard` publishes
`geometry_msgs/msg/Twist` on `/cmd_vel`; no remapping is required.

## Keyboard controls

```text
u  i  o
j  k  l
m  ,  .
```

- `i`: forward
- `,`: backward
- `j` / `l`: strafe left / right
- `u`, `o`, `m`, `.`: diagonal motion
- `q` / `e`: rotate counter-clockwise / clockwise
- `k` or `Space`: stop immediately

For this Omni4 firmware, motor order is left-front, left-back, right-back,
right-front. If a direction is wrong, stop the robot and correct motor wiring
or the wheel-direction configuration before further driving.

## Verify the command path

Run these in a fourth sourced terminal while pressing a teleop key:

```bash
ros2 topic info /cmd_vel
ros2 topic echo /irob_motor_cmd
```

`/cmd_vel` should have `teleop_twist_keyboard` as a publisher and
`iRob_controller` as a subscriber. `/irob_motor_cmd` should change while a
movement key is pressed.

## Common problems

| Problem | Check |
| --- | --- |
| `/dev/ESP32` does not exist | Reconnect the board and install the udev rule in the “Before driving” section. |
| Interface cannot open the serial port | Ensure no packet-test program or serial monitor is using `/dev/ESP32`; confirm your user belongs to the `dialout` group. |
| Keyboard keys do not move the robot | Keep the teleop terminal focused, then run `ros2 topic info /cmd_vel` and confirm the controller is running. |
| Robot moves in the wrong direction | Press `Space` immediately. Verify the Omni4 motor order and wheel directions before trying again. |
| Robot does not stop after closing teleop | Use the physical emergency stop or remove motor power; do not rely on a closed terminal as a safety stop. |

## Safety

Lift the wheels off the ground for the first test. Keep a hand near the power
disconnect or emergency stop. Always press `Space` or `k` before switching
terminals or stopping a ROS node.
