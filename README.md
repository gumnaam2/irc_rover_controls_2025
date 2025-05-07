# irc_rover_controls_2025
This repository is a collection of programs written for execution of various fuctionalities of the rover. It is to also serve as destination for the progress made until IRC 2025 regarding the control of the rover iteration AMARAN.

# Steps to start rover

1. Clone this repository

2. Sign in to the tezant network: assign an IPv4 address to yourself, gateway 192.168.69.100.

3. `ros2 run joy joy_node` in another terminal (on your computer) (with the joystick connected). This will publish the joystick values on the` /joy` topic.

4. On another terminal (on your computer) run the required package from drive_control or arm_control: for just driving, `ros2 run differential joystick_innerrr` from the drive_control directory; for controlling joints individually, `ros2 run joint_control full_command` from arm_control. These read the joystick data from the `joy` topic and publish the PWM values for the different motors on the required topics.
 
5. Start the rover and `ssh username@ip` (of the rover) in one terminal. From this launch DriveLaunch and/or ArmLaunch. These read the PWM values published on the /drive_commands or the shoulder/elbow/arm topics and send the required signals to the nodes through CAN.

Alternatively (after step 2), install terminator and in a terminal (not terminator) run `./launch.sh username@ip` (username, ip of the rover). 