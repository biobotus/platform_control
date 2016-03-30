# Platform Control

This ROS package contains multiple nodes:
  - motor_control_xy
  - motor_control_z
  - motor_control_sp
  - motor_control_mp

Requirements to run:
  - Having installed [pigpio] and having executed `sudo pigpiod`.

Axis control (motors x0, x1, y, z0, z1 and z2) is done on a Raspberry Pi 2.
Pipettes control (single pipette - sp - and multiple pipette - mp) is done on another Raspberry Pi 2.

**Running axis control nodes**
  - `roslaunch platform_control axis_control.launch`
  - `rosrun platform_control hmi_control.py` (optional - in a second terminal)

**Running axis control nodes**
  - `roslaunch platform_control pipette_control.launch`


[pigpio]: http://abyz.co.uk/rpi/pigpio/python.html
