import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from msg_interfaces.msg import ArmEndMotion
from msg_interfaces.msg import EncoderArm  # Replace with your custom message type

from sensor_msgs.msg import Joy

import time
import numpy as np

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_angle_controller')

        # Parameters
        self.declare_parameter('velocity_threshold', 35)  # Threshold for switching to position hold
        self.declare_parameter('kp', 2)  # Proportional gain
        self.declare_parameter('ki', 0)  # Integral gain
        self.declare_parameter('kd', 0.025)  # Derivative gain
        self.declare_parameter('control_rate', 50)  # Control loop frequency (Hz)

        # Get parameters
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.control_rate = self.get_parameter('control_rate').value

        # PID variables
        self.setpoint = None  # Target angular position to hold
        self.last_error = 0.0
        self.integral = []
        self.last_time = None
        self.control_active = False

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            ArmEndMotion, 'arm_commands', self.cmd_vel_callback, 10)
        self.encoder_sub = self.create_subscription(
            EncoderArm,'encoder_arm', self.encoder_callback, 10)

        # Publisher
        self.control_pub = self.create_publisher(ArmEndMotion, 'arm_commands_pid_shoulder', 10)

        # Timer for PID control loop
        self.timer = self.create_timer(1.0 / self.control_rate, self.pid_control_loop)

        # create subscription to joystick inputs
        self.joystick_subscription = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)

        self.joy_activated = False

        # State variables
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.velocity_command = 0.0
        self.is_setpoint=False

        self.get_logger().info('PID Angle Controller Node started:Shoulder')

    def cmd_vel_callback(self, msg):
        """
        Callback for velocity command input.
        """
        self.velocity_command = msg.speed[0]
        # Check if velocity is below threshold to activate position hold
        #if abs(self.velocity_command) < self.velocity_threshold and self.is_setpoint is False:
        if (not self.joy_activated) and self.is_setpoint is False:
            self.is_setpoint=True
            if self.setpoint!=self.current_position:
                self.get_logger().info(f'Setpoint updated:{self.current_position}:Shoulder')
            self.setpoint = self.current_position  # Hold current position

            if abs(self.current_velocity)>0:
                if not self.control_active:
                    self.get_logger().info('Activating position hold:Shoulder')
                    self.integral = []  # Reset integral term
                    self.last_error = 0.0
                    self.last_time = time.time()
                    self.control_active = True
        #elif self.is_setpoint and abs(self.velocity_command)>self.velocity_threshold:
        elif self.is_setpoint and  self.joy_activated:
            # Deactivate position hold
            self.is_setpoint=False
            if self.control_active:
                self.get_logger().info('Deactivating position hold:Shoulder')
                self.control_active = False

    def encoder_callback(self, msg):
        """
        Callback for encoder feedback (position and velocity).
        """
        self.current_position = msg.arm_node0[0]
        self.current_velocity = msg.arm_node0[1]

    def joystick_callback(self, msg):
        a_butt=msg.buttons[0]
        b_butt=msg.buttons[1]
        x_butt=msg.buttons[2]
        y_butt=msg.buttons[3]
        lb_butt=msg.buttons[4]
        rb_butt=msg.buttons[5]
        start_butt=msg.buttons[7]

        if (a_butt != 0) or (b_butt != 0) or (x_butt != 0) or (y_butt != 0):
            self.joy_activated = True
        else:
            self.joy_activated = False

    def pid_control_loop(self):
            """
            Main PID control loop.
            """
            if not self.control_active or self.setpoint is None:
                return  # Skip control if not active

            # Calculate time delta
            current_time = time.time()
            dt = current_time - self.last_time if self.last_time else 1.0 / self.control_rate
            self.last_time = current_time

            # Calculate PID error terms
            error = self.setpoint - self.current_position
            self.integral.append(error * dt)
            
            if len(self.integral)>500:
                deleted_value=self.integral.pop(0)

            integral=np.sum(np.array(self.integral))
            derivative = (error - self.last_error) / dt if dt > 0 else 0.0
            self.last_error = error

            # Compute control effort (velocity command)
            control_effort = (self.kp * error) + (self.ki * integral) + (self.kd * derivative)
            if control_effort<0:
                reverse=0
            else:
                reverse=1
            control_effort=abs(int(control_effort))
            if control_effort>200:
                control_effort=200                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
            # Publish control effort
            control_msg = ArmEndMotion()
            control_msg.speed = [control_effort]
            control_msg.direction = [reverse]
            self.control_pub.publish(control_msg)

        # Logging (optional)
        #self.get_logger().info(f'Setpoint: {self.setpoint:.3f}, Position: {self.current_position:.3f}, '
                               #f'Control Effort: {control_effort:.3f}')


def main(args=None):
    rclpy.init(args=args)
    pid_node = PIDController()
    try:
        rclpy.spin(pid_node)
    except KeyboardInterrupt:
        pid_node.get_logger().info('Shutting down PID Angle Controller Node:Shoulder')
    finally:
        pid_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()