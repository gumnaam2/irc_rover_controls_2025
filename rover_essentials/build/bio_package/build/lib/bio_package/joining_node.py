import rclpy
from rclpy.node import Node

# Example message types
from std_msgs.msg import String,Int32
from sensor_msgs.msg import LaserScan

from msg_interfaces.msg import ArmEndMotion

# In this example, we’ll publish a String message regardless of the input type.
# You can replace with other message types if needed.

class JoiningNode(Node):
    def __init__(self):
        super().__init__('joining_node')

        # Current selected topic. Can be 'topic1' or 'topic2'.
        self.selected_topic = None
        
        # Declare parameters if you want them in ROS 2 param server (optional)
        # self.declare_parameter('topic1_name', 'topic1')
        # self.declare_parameter('topic2_name', 'topic2')
        # self.declare_parameter('command_topic', 'topic_command')
        # self.declare_parameter('output_topic', 'topic_output')
        
        # For simplicity, hard-code topic names here or use parameters.
        topic1_name = 'arm_commands_full_commands'
        topic2_name = 'arm_commands_bio_pid'
        command_topic = 'switching_command'
        output_topic = 'arm_commands'

        # Subscribers
        self.topic1_sub = self.create_subscription(
            ArmEndMotion,
            topic1_name,
            self.full_callback,
            10
        )
        self.topic2_sub = self.create_subscription(
            ArmEndMotion,
            topic2_name,
            self.bio_callback,
            10
        )
        self.command_sub = self.create_subscription(
            Int32,
            command_topic,
            self.command_callback,
            10
        )

        # Publisher
        self.output_pub = self.create_publisher(
            ArmEndMotion,
            output_topic,
            10
        )

        self.get_logger().info("JoiningNode started. Waiting for commands to select topic1 or topic2.")

    def command_callback(self, msg):
        """
        Update the selected topic based on the command message.
        The message could be "topic1" or "topic2", for example.
        """
        command = msg.data
        if command==0:
            self.selected_topic = 'full'
            self.get_logger().info(f"Selected topic is now: {self.selected_topic}")
        elif command==1:
            self.selected_topic ='bio'
            self.get_logger().info(f'Selected topic is now: {self.selected_topic}')
        else:
            self.get_logger().warn(f"Received unknown command: {command}. Ignoring.")

    def full_callback(self, msg):
        """
        Called whenever a new message is received on topic1.
        Check if topic1 is the selected topic; if so, publish to output.
        """
        if self.selected_topic == 'full':
            out_msg = ArmEndMotion()
            # Here is where you might do any necessary formatting.
            # For illustration, just pass the string data with a prefix.
            out_msg.speed = msg.speed
            out_msg.direction=msg.direction
            out_msg.sys_check=msg.sys_check
            out_msg.reset=msg.reset
            self.output_pub.publish(out_msg)
            self.get_logger().debug(f"Published from full: {out_msg.speed}")

    def bio_callback(self, msg):
        """
        Called whenever a new message is received on topic2.
        Check if topic2 is the selected topic; if so, publish to output.
        """
        if self.selected_topic == 'bio':
            out_msg = ArmEndMotion()
            # Here is where you might do any necessary formatting.
            # For illustration, just pass the string data with a prefix.
            out_msg.speed = msg.speed
            out_msg.direction=msg.direction
            self.output_pub.publish(out_msg)
            self.get_logger().debug(f"Published from bio: {out_msg.speed}")



def main(args=None):
    rclpy.init(args=args)
    node = JoiningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

