import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from msg_interfaces.msg import ArmEndMotion
from adafruit_servokit import ServoKit
from time import sleep

kit = ServoKit(channels=16)
SERVO_CHANNEL = 0  
kit.servo[SERVO_CHANNEL].set_pulse_width_range(500,2500)
MIN_ANGLE = (90/255)*180
MAX_ANGLE = (190/255)*180

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_testing')
        self.gripper_cmd_subscriber = self.create_subscription( ArmEndMotion,'arm_commands', self.joystick_callback, 10)
        self.get_logger().info('GripperControlNode has been initialized.')

    def joystick_callback(self, cmd): #cmd type [,,,,,int64]

        # Ensure the speed list is not empty
        if not cmd.speed:
            self.get_logger().warn('Received ArmEndMotion message with empty speed list.')
            return

        # Get the last speed value from the command
        val = cmd.speed[-1]
        self.get_logger().info(f'Received speed value: {val}')
        kit.servo[SERVO_CHANNEL].angle = max(MIN_ANGLE, min(MAX_ANGLE, (val/255)*180))
        # sleep(0.1)
        self.get_logger().info(f'Turned to : {max(MIN_ANGLE, min(MAX_ANGLE, (val/255)*180))}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GripperControlNode has been interrupted and is shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

