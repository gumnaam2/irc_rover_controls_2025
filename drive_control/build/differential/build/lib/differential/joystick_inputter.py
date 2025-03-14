#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from msg_interfaces.msg import Drive
toggle = False
# system_check_memory = [False for _ in range(100)]

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')

        # Create subscription to joystick inputs
        self.joystick_subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10)

        # Create publisher for drive commands
        self.drive_publisher = self.create_publisher(Drive, 'drive_commands', 10)

    def joystick_callback(self, joyos):
        global toggle
        drive_command = Drive()
        # Extract joystick axes
        left_hor = joyos.axes[0]
        left_ver = joyos.axes[1]

        cross_hor = joyos.axes[6]

        # Initialize drive variables
        drive_speed_right = 0
        drive_direction_right = 0
        drive_speed_left = 0
        drive_direction_left = 0

        # toggle = False

        # if toggle == False:
        #     if start_button:
        #         drive_command.sys_check = True
        #         toggle = True
        #     else:
        #         drive_command.sys_check = False
        # else: 
        #     if not(start_button):
        #         toggle=False
        #     drive_command.sys_check = False
        


        # if not (True in system_check_memory):
        #     if start_button:
        #         drive_command.sys_check = True
        #     else:
        #         drive_command.sys_check = False
        #     system_check_memory.append(drive_command.sys_check)
        #     system_check_memory.pop(0)
        # else:
        #     system_check_memory.append(False)
        #     system_check_memory.pop(0)

        if cross_hor == 1:
            toggle = True
        if cross_hor == -1:
            toggle = False

        drive_command.sys_check = toggle

        # Forward
        if left_ver > 0:  
            if left_ver <= 0.1:
                drive_speed_right = 0
                drive_direction_right = 1
                drive_speed_left = 0
                drive_direction_left = 1
            else:
                drive_speed_right = (left_ver - 0.1) / 1.0
                drive_direction_right = 1
                drive_speed_left = (left_ver - 0.1) / 1.0
                drive_direction_left = 1

        # Backward
        elif left_ver < 0:  
            if abs(left_ver) <= 0.1:
                drive_speed_right = 0
                drive_direction_right = 0
                drive_speed_left = 0
                drive_direction_left = 0
            else:
                drive_speed_right = (abs(left_ver) - 0.1) / 1.0
                drive_direction_right = 0
                drive_speed_left = (abs(left_ver) - 0.1) / 1.0
                drive_direction_left = 0

        # Right turn
        elif left_hor < 0:  
            if abs(left_hor) <= 0.1:
                drive_speed_right = 0
                drive_direction_right = 0
                drive_speed_left = 0
                drive_direction_left = 1
            else: 
                drive_speed_right = (abs(left_hor) - 0.1) / 1.0
                drive_direction_right = 0
                drive_speed_left = (abs(left_hor) - 0.1) / 1.0
                drive_direction_left = 1

        # Left turn
        elif left_hor > 0:  
            if left_hor <= 0.1:
                drive_speed_right = 0
                drive_direction_right = 1
                drive_speed_left = 0
                drive_direction_left = 0
            else:
                drive_speed_right = (left_hor - 0.1) / 1.0
                drive_direction_right = 1
                drive_speed_left = (left_hor - 0.1) / 1.0  
                drive_direction_left = 0

        # Publish the drive command
        speed_list = [drive_speed_left, drive_speed_left, drive_speed_left, drive_speed_right, drive_speed_right, drive_speed_right]
        drive_command.speed = [int(element * 255) for element in speed_list]  # Convert to PWM range
        drive_command.direction = [drive_direction_left, drive_direction_left, drive_direction_left, drive_direction_right, drive_direction_right, drive_direction_right]

        self.drive_publisher.publish(drive_command)
        self.get_logger().info(f'Published drive command - Speed: {list(drive_command.speed)}, Direction: {list(drive_command.direction)}, System Check Request: {drive_command.sys_check}')


def main(args=None):
    rclpy.init(args=args)

    drive_controller = DriveController()

    try:
        rclpy.spin(drive_controller)
    except KeyboardInterrupt:
        print("Shutting down due to KeyboardInterrupt")
    finally:
        drive_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
