import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from adafruit_servokit import ServoKit


SERVO_CHANNEL = 15               

class ServoAngleSubscriber(Node):
    def __init__(self):
        super().__init__('servo_angle_subscriber')
        self.kit = ServoKit(channels=16)
        self.kit.servo[SERVO_CHANNEL].set_pulse_width_range(500,2500)
        self.subscription = self.create_subscription(Int32,'/camera/servo_angle',self.listener_callback,10)
        self.subscription 

    def listener_callback(self, msg: Int32):
        angle = msg.data
        angle = max(0, min(angle, 180))
        mapped_angle = ((180-angle) *255.0) / 180.0
        self.kit.servo[SERVO_CHANNEL].angle = mapped_angle
        self.get_logger().info(f"Received angle={angle}, mapped_angle={mapped_angle:.1f}")

def main(args=None):
    rclpy.init(args=args)
    servo_angle_subscriber = ServoAngleSubscriber()
    rclpy.spin(servo_angle_subscriber)

    # Clean up
    servo_angle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
