# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from msg_interfaces.msg import Drive  # Replace 'custom_msgs' with the actual package name

# class DifferentialDriveNode(Node):
#     def __init__(self):
#         super().__init__('differential_drive_node')
#         self.get_logger().info('Autonomous diff-drive control started')

#         # Parameters
#         self.declare_parameter('wheel_radius', 0.12)  # meters
#         self.declare_parameter('wheel_separation', 0.5)  # meters
#         self.declare_parameter('max_motor_speed', 5.236)  # rad/s

#         self.wheel_radius = self.get_parameter('wheel_radius').value
#         self.wheel_separation = self.get_parameter('wheel_separation').value
#         self.max_motor_speed = self.get_parameter('max_motor_speed').value

#         self.max_rover_vel=self.wheel_radius*self.max_motor_speed
#         self.max_rover_w=self.max_rover_vel*2/self.wheel_separation

#         # Subscriber and Publisher
#         self.subscription = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_vel_callback,
#             10
#         )

#         self.publisher = self.create_publisher(
#             Drive,
#             'drive_commands',
#             10
#         )

#     def cmd_vel_callback(self, msg):
#         linear_velocity = msg.linear.x*self.max_rover_vel
#         angular_velocity = msg.angular.z*self.max_rover_w

#         # Compute wheel speeds
#         v_right = linear_velocity + (self.wheel_separation / 2.0) * angular_velocity
#         v_left = linear_velocity - (self.wheel_separation / 2.0) * angular_velocity

#         # Convert to angular velocity of wheels (rad/s)
#         omega_right = v_right / self.wheel_radius
#         omega_left = v_left / self.wheel_radius

#         # Clamp speeds to max_motor_speed and find the ratio of them to max motor speeds
#         omega_right = max(min(omega_right, self.max_motor_speed), -self.max_motor_speed)/self.max_motor_speed
#         omega_left = max(min(omega_left, self.max_motor_speed), -self.max_motor_speed)/self.max_motor_speed

#         #Generate PWM values assuming 255 is max motor speed
#         omega_right_pwm=int(omega_right*255)
#         omega_left_pwm=int(omega_left*255)
#         if omega_left_pwm!=omega_right_pwm:
#             self.get_logger().info('Different PWMs for two sides')
#         if omega_right_pwm!=0:
#             omega_right_dir=int(((omega_right_pwm/abs(omega_right_pwm))+1)/2)
#         else:
#             omega_right_dir=0
#         if omega_left_pwm!=0:
#             omega_left_dir=int(((omega_left_pwm/abs(omega_left_pwm))+1)/2)
#         else:
#             omega_left_dir=0
#         # if omega_right_dir!=omega_left_dir:
#         #     omega_right_pwm=omega_right_pwm+100
#         #     omega_left_pwm=omega_left_pwm+100
#         if abs(omega_left_pwm)>255:
#             omega_left_pwm=255
#         if abs(omega_right_pwm)>255:
#             omega_right_pwm=255
#         # Distribute speeds to motors
#         motor_speeds = Drive()
#         motor_speeds.speed = [abs(omega_left_pwm)] * 3 + [abs(omega_right_pwm)] * 3
#         motor_speeds.direction=[omega_left_dir]*3 + [omega_right_dir]*3

#         # Publish the motor speeds
#         self.publisher.publish(motor_speeds)

#         self.get_logger().info(f"Published motor speeds: {motor_speeds.speed}")
#         self.get_logger().info(f"Published motor directions:{motor_speeds.direction}")
    
#     def reset(self):
#         reset_speed=Drive()
#         reset_speed.speed=[0,0,0,0,0,0]
#         reset_speed.direction=[0,0,0,0,0,0]
#         self.publisher.publish(reset_speed)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DifferentialDriveNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.reset()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import signal
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from msg_interfaces.msg import Drive  # Replace 'custom_msgs' with the actual package name

# IMPORTANT: We need to import the Executor
from rclpy.executors import SingleThreadedExecutor
import time


class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('differential_drive_node')
        self.get_logger().info('Autonomous diff-drive control started')

        # Parameters
        self.declare_parameter('wheel_radius', 0.12)  # meters
        self.declare_parameter('wheel_separation', 0.5)  # meters
        self.declare_parameter('max_motor_speed', 5.236)  # rad/s

        self.msg_delay_tolerance = 2.0

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_motor_speed = self.get_parameter('max_motor_speed').value

        self.max_rover_vel=self.wheel_radius*self.max_motor_speed
        self.max_rover_w=self.max_rover_vel*2/self.wheel_separation

        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.current_vel_command = None

        self.publisher = self.create_publisher(
            Drive,
            'drive_commands',
            10
        )

        timer_period = 0.1 # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.publish_velocity)

    def sigint_handler(self, signal, frame):
        stop_cmd = FinalVel()
        stop_cmd.speed = 0
        stop_cmd.direction = 0

        sys.exit(0)

    def cmd_vel_callback(self,msg):
        self.current_vel_command = {"timestamp": self.get_clock().now().seconds_nanoseconds()[0], "msg": msg}


    def publish_velocity(self):

        if not self.current_vel_command:

            self.get_logger().warn("No Velocity Data Received!!")
            linear_velocity = 0
            angular_velocity = 0
        
        else:

            msg_time_s = self.current_vel_command["timestamp"]
            current_time_s = self.get_clock().now().seconds_nanoseconds()[0]
            
            if abs(msg_time_s - current_time_s) > self.msg_delay_tolerance:
                self.get_logger().warn("Velocity Data Not Received From a Long Time!!")
                linear_velocity = 0
                angular_velocity = 0
            else:
                msg = self.current_vel_command["msg"]
                linear_velocity = msg.linear.x*self.max_rover_vel
                angular_velocity = msg.angular.z*self.max_rover_w

        # Compute wheel speeds
        v_right = linear_velocity + (self.wheel_separation / 2.0) * angular_velocity
        v_left = linear_velocity - (self.wheel_separation / 2.0) * angular_velocity

        # Convert to angular velocity of wheels (rad/s)
        omega_right = v_right / self.wheel_radius
        omega_left = v_left / self.wheel_radius

        # Clamp speeds to max_motor_speed and find the ratio of them to max motor speeds
        omega_right = max(min(omega_right, self.max_motor_speed), -self.max_motor_speed)/self.max_motor_speed
        omega_left = max(min(omega_left, self.max_motor_speed), -self.max_motor_speed)/self.max_motor_speed

        #Generate PWM values assuming 255 is max motor speed
        omega_right_pwm=int(omega_right*255)
        omega_left_pwm=int(omega_left*255)
        if omega_left_pwm!=omega_right_pwm:
            self.get_logger().info('Different PWMs for two sides')
        if omega_right_pwm!=0:
            omega_right_dir=int(((omega_right_pwm/abs(omega_right_pwm))+1)/2)
        else:
            omega_right_dir=0
        if omega_left_pwm!=0:
            omega_left_dir=int(((omega_left_pwm/abs(omega_left_pwm))+1)/2)
        else:
            omega_left_dir=0
        # if omega_right_dir!=omega_left_dir:
        #     omega_right_pwm=omega_right_pwm+100
        #     omega_left_pwm=omega_left_pwm+100
        if abs(omega_left_pwm)>255:
            omega_left_pwm=255
        if abs(omega_right_pwm)>255:
            omega_right_pwm=255
        # Distribute speeds to motors
        motor_speeds = Drive()
        motor_speeds.speed = [abs(omega_left_pwm)] * 3 + [abs(omega_right_pwm)] * 3
        motor_speeds.direction=[omega_left_dir]*3 + [omega_right_dir]*3

        # Publish the motor speeds
        self.publisher.publish(motor_speeds)

        self.get_logger().info(f"Published motor speeds: {motor_speeds.speed}")
        self.get_logger().info(f"Published motor directions:{motor_speeds.direction}")
    
    def reset(self):
        reset_speed=Drive()
        reset_speed.speed=[0,0,0,0,0,0]
        reset_speed.direction=[0,0,0,0,0,0]
        self.publisher.publish(reset_speed)

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()
    signal.signal(signal.SIGINT, node.sigint_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
