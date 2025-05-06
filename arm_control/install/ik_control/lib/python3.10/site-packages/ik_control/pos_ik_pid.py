#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from msg_interfaces.msg import TargetPose
from msg_interfaces.msg import EncoderArm
from msg_interfaces.msg import ArmEndMotion, ArmAngle

import matplotlib.pyplot as plt

import math
import numpy as np

import time

switch = False
motion_cmd = ArmEndMotion()
joy_activated = False

target_encoders_shld=[]
current_encoders_shld=[]
index=[]
index_counter=0
target_encoders_elb=[]
current_encoders_elb=[]
shoulder_pwm=[]
elbow_pwm=[]
pwm_index=[]
pwm_index_counter=0
class JointPositionPIDNode(Node):
    def __init__(self):
        super().__init__('joint_position_pid_node')

        # Declare PID parameters for each joint (example for 3 joints).
        # If you have more joints, extend the arrays.
        # Gains can be tuned individually. [base, shoulder, elbow]
        self.kp = [1.0, 1.1, 1.0] 
        self.ki = [0.0, 0.0, 0.0]
        self.kd = [0.0, 0.0, 0.0]

        # Tolerance for each joint (in encoder counts or whatever units).
        self.tolerance = [50, 0, 0]

        # Maximum PWM (for the "full speed" scenario).
        self.pwm_max = 220
        # Minimum PWM that you still want to apply (can be 0).
        self.pwm_min = 30

        # If the error is large, we go full PWM (255).
        # Let's define a threshold beyond which we consider the joint "far away".
        # If |error| > big_error_threshold[joint], we run 255 until we get closer.
        self.big_error_threshold = [1000, 1000, 1000]

        # Initialize states for each joint
        # We store integral error and previous error for derivative
        self.integral_error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]

        # Control loop frequency (50 Hz)
        self.control_frequency = 250.0
        self.dt = 1.0 / self.control_frequency

        # Last known target (from TargetPose) and current (from EncoderArm)
        self.target_encoders = [0, 0, 0]  # base, shoulder, elbow
        self.current_encoders = [0, 0, 0]

        self.gripper_state = 255

        self.joy_activated = False

        self.base_speed = 0
        self.base_direction = 1

        self.left_motor_speed = 0
        self.left_motor_direction = 1

        self.right_motor_speed = 0
        self.right_motor_direction = 1

        self.shoulder_alpha = 90
        self.elbow_beta = 90

        self.wrist_l_cmd = 0
        self.wrist_r_cmd = 0

        self.wrist_l_dir = 0
        self.wrist_r_dir = 0

        # Create subscriptions
        self.target_pose_sub = self.create_subscription(
            TargetPose,
            'target_encoder',
            self.target_pose_callback,
            10
        )
        self.encoder_sub = self.create_subscription(
            EncoderArm,
            'encoder_arm',
            self.encoder_callback,
            10
        )

        # Create subscription to joystick inputs
        self.joystick_subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10
        )

        # Create publisher for the motor commands
        self.motion_pub = self.create_publisher(ArmEndMotion, 'arm_commands', 10)
        self.angle_pub = self.create_publisher(ArmAngle, '/arm_angle', 10)

        # Create a timer for the control loop
        self.control_timer = self.create_timer(self.dt, self.update_wrist_cmd)

        self.wrist_receiver = self.create_subscription(ArmEndMotion, 'arm_commands_wrist_pid', self.wrist_callback, 10)

        # Create a timer for the control loop
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        # self.get_logger().info("JointPositionPIDNode started.")

    def target_pose_callback(self, msg: TargetPose):
        """
        Callback for receiving the target encoder positions from some external source.
        E.g., msg.base, msg.shoulder, msg.elbow, etc.
        """
        self.target_encoders[0] = msg.base
        self.target_encoders[1] = msg.shoulder
        self.target_encoders[2] = msg.elbow
        # If you have more joints, extend accordingly.

    def encoder_callback(self, msg: EncoderArm):
        """
        Callback for receiving the current joint encoders from the hardware.
        E.g., msg.base, msg.shoulder, msg.elbow, etc.
        """
        self.current_encoders[0] = msg.arm_node2[0]
        self.current_encoders[1] = msg.arm_node0[0]
        self.current_encoders[2] = msg.arm_node1[0]
        # Extend if more joints are in the message.

        self.shoulder_alpha = 1.57 - ((self.current_encoders[1]*3.1415)/17330)
        self.elbow_beta = 1.57 - ((self.current_encoders[2]*3.1415)/9405)


    def joystick_callback(self, msg: Joy):
        global switch
        global joy_activated

        start_butt=msg.buttons[7]
        back_button = msg.buttons[6]
        a_butt=msg.buttons[0]
        b_butt=msg.buttons[1]
        x_butt=msg.buttons[2]
        y_butt=msg.buttons[3]
        lb_butt=msg.buttons[4]
        rb_butt=msg.buttons[5]

        cross_ver = msg.axes[7]
        cross_hor = msg.axes[6]

        right_hor = msg.axes[3]
        right_ver = msg.axes[4]

        if start_butt:
            switch = True
        if back_button:
            switch = False

        # if cross_ver == 1:
        #     self.gripper_state = 0

        # if cross_ver == -1:
        #     self.gripper_state = 255

        # if cross_ver == 1:
        #     self.target_encoders[1] += 200
        #     self.target_encoders[2] += 200
        #     time.sleep(1)
        #     self.target_encoders[1] -= 200
        #     self.target_encoders[2] -= 200
        #     time.sleep(1)

        # if cross_hor == 1:
        #     self.left_motor_speed = 120
        #     self.left_motor_direction = 0

        #     self.right_motor_speed = 120
        #     self.right_motor_direction = 0

        # elif cross_hor == 0:
        #     self.left_motor_speed = 0
        #     self.left_motor_direction = 0

        #     self.right_motor_speed = 0
        #     self.right_motor_direction = 1

        # elif cross_hor == -1:
        #     self.left_motor_speed = 120
        #     self.left_motor_direction = 1

        #     self.right_motor_speed = 120
        #     self.right_motor_direction = 1 


        if right_hor < 0:  
            if abs(right_hor) <= 0.1:
                self.base_speed = 0
                self.base_direction = 1
            else: 
                self.base_speed = int(((abs(right_hor) - 0.1) / 0.9)*255)
                self.base_direction = 1


        elif right_hor > 0:  
            if right_hor <= 0.1:
                self.base_speed = 0
                self.base_direction = 0

            else:
                self.base_speed = int(((right_hor - 0.1) / 0.9)*255) 
                self.base_direction = 0 
        else:
            self.base_direction = 0
            self.base_speed = 0 

        # self.get_logger().info(f"{a_butt}, {b_butt}, {x_butt}, {y_butt}, {(a_butt != 0) or (b_butt != 0) or (x_butt != 0) or (y_butt != 0)}")

        if (a_butt != 0) or (b_butt != 0) or (x_butt != 0) or (y_butt != 0) or (lb_butt != 0) or (rb_butt != 0) or (cross_ver != 0):
            self.joy_activated = True
        else:
            self.joy_activated = False

    def update_wrist_cmd(self):
        self.shoulder_alpha = 1.57 - (((self.current_encoders[1])*3.1415)/17330)
        self.elbow_beta = 3.1415 - (1.57 - (((self.current_encoders[2])*3.1415)/(2*9405)))
        angle_update = ArmAngle()
        angle_update.alpha = float(self.shoulder_alpha)
        angle_update.beta = float(self.elbow_beta)

        self.angle_pub.publish(angle_update)

    def wrist_callback(self, msg: ArmEndMotion):
        self.wrist_l_cmd = msg.speed[3]
        self.wrist_r_cmd = msg.speed[4]

        self.wrist_l_dir = msg.direction[3]
        self.wrist_r_dir = msg.direction[4]

    def control_loop(self):
        global switch
        global motion_cmd
        global joy_activated

        #For plotting
        global target_encoders_shld,target_encoders_elb,current_encoders_elb,current_encoders_shld,index,index_counter,shoulder_pwm,elbow_pwm,pwm_index,pwm_index_counter
        """
        Periodic PID control loop (runs at ~50 Hz).
        """
        speeds = [0, 0, 0]
        directions = [0, 0, 0]
        target_encoders_elb.append(self.target_encoders[2])
        target_encoders_shld.append(self.target_encoders[1])
        index.append(index_counter)
        index_counter=index_counter+1
        current_encoders_elb.append(self.current_encoders[2])
        current_encoders_shld.append(self.current_encoders[1])

        for i in range(3):  # for base, shoulder, elbow
            error = self.target_encoders[i] - self.current_encoders[i]
            abs_error = abs(error)

            # Check tolerance
            if abs_error <= self.tolerance[i]:
                # Within tolerance => no movement needed
                speeds[i] = 40
                directions[i] = 1
                # Optionally reset integrator
                self.integral_error[i] = 0.0
                self.prev_error[i] = 0.0        # self.get_logger().info("JointPositionPIDNode started.")
                continue

            # If the error is large => drive full speed (255)
            if abs_error > self.big_error_threshold[i]:
                pwm_value = self.pwm_max
            else:
                # Do the normal PID computation
                p_term = self.kp[i] * error
                self.integral_error[i] += error * self.dt
                i_term = self.ki[i] * self.integral_error[i]
                d_term = self.kd[i] * ((error - self.prev_error[i]) / self.dt)

                pid_output = p_term + i_term + d_term

                # Clip to [pwm_min, pwm_max]
                pwm_value = int(abs(pid_output))
                pwm_value = max(pwm_value, self.pwm_min)
                pwm_value = min(pwm_value, self.pwm_max)

            # Determine direction
            directions[i] = 1 if (error > 0) else 0
            if switch:
                speeds[i] = pwm_value
            else:
                speeds[i] = 0

            # Update prev_error
            self.prev_error[i] = error

        angle_update = ArmAngle()
        angle_update.alpha = float(self.shoulder_alpha)
        angle_update.beta = float(self.elbow_beta)

        self.angle_pub.publish(angle_update)

        # Publish the commands
        # if self.joy_activated:
        shoulder_pwm.append(speeds[1])
        elbow_pwm.append(speeds[2])
        pwm_index.append(pwm_index_counter)
        pwm_index_counter=pwm_index_counter+1
        motion_cmd.speed = [speeds[1], speeds[2], self.base_speed, self.wrist_l_cmd, self.wrist_r_cmd, self.gripper_state]   # e.g. [speed_base, speed_shoulder, speed_elbow]
        motion_cmd.direction = [directions[1], directions[2], self.base_direction, self.wrist_l_dir, self.wrist_r_dir, 0]  # e.g. [dir_base, dir_shoulder, dir_elbow]

        self.motion_pub.publish(motion_cmd)
        if switch:
            self.get_logger().info("PID, joy_activated")
        else:
            self.get_logger().info("Stopped, joy_activated")

        # else:
        #     motion_cmd.speed = [0, 0, self.base_speed, 0, 0, 255]   # e.g. [speed_base, speed_shoulder, speed_elbow]
        #     motion_cmd.direction = [1, 1, self.base_direction, 1, 1, 1]  # e.g. [dir_base, dir_shoulder, dir_elbow]

            self.motion_pub.publish(motion_cmd)
            if switch:
                self.get_logger().info("PID, joy_deactivated")
            else:
                self.get_logger().info("Stopped, joy_deactivated")


        # (Optional) debug log
        # self.get_logger().info(
        #     f"PID: targets={self.target_encoders}, current={self.current_encoders}, "
        #     f"speeds={speeds}, dirs={directions}"
        # )

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionPIDNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # plt.plot(index, target_encoders_shld, label="Target Shoulder")
        # plt.plot(index, current_encoders_shld, label="Actual Shoulder")
        # plt.legend()
        # plt.show()

        # plt.plot(index, target_encoders_elb, label="Target Elbow")
        # plt.plot(index, current_encoders_elb, label="Actual Elbow")
        # plt.legend()
        # plt.show()

        # plt.plot(pwm_index,shoulder_pwm,label="Shoulder PWM")
        # plt.show()

        # plt.plot(pwm_index,elbow_pwm,label="Elbow PWM")
        # plt.show()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
