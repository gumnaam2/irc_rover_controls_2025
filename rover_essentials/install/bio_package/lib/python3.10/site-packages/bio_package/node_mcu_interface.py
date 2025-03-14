import rclpy
from rclpy.node import Node
import requests
from sensor_msgs.msg import Joy


class NodeMCUClient(Node):
    """
    A ROS2 node that periodically sends an HTTP GET request to a NodeMCU server
    and logs the response. You can adapt endpoints to send commands like
    '/blue', '/green', '/red', or query '/temperature', '/humidity', etc.
    """

    def __init__(self):
        super().__init__('node_mcu_client')
        # Declare and retrieve parameters
        self.declare_parameter('nodemcu_ip', '192.168.69.95')
        self.declare_parameter('command', 'open')  # which command to send by default
        self.declare_parameter('update_rate', 1.0) # how often (in seconds) we query

        self.nodemcu_ip = self.get_parameter('nodemcu_ip').value
        self.command = self.get_parameter('command').value
        self.update_rate = self.get_parameter('update_rate').value

        # Create a periodic timer to send requests
        self.timer = self.create_timer(self.update_rate, self.timer_callback)

        self.joystick_sub=self.create_subscription(Joy,'/joy',self.joystick_callback,10)
        
        self.get_logger().info(f"NodeMCUClient started. IP: {self.nodemcu_ip}, "
                               f"Command: {self.command}, Rate: {self.update_rate}s")


    def joystick_callback(self,joystick):
        if joystick.axes[7]==1.0:
            self.command='close'
        elif joystick.axes[7]==-1.0:
            self.command='open'
    def timer_callback(self):
        """
        Called periodically to send an HTTP GET to the NodeMCU.
        For example: http://192.168.69.95/blue
        Then logs the response text or any error.
        """
        url = f'http://{self.nodemcu_ip}/{self.command}'
        #self.get_logger().info(f"Sending request to: {url}")
        try:
            response = requests.get(url, timeout=2.0)  # 2-second timeout
            if response.status_code == 200:
                self.get_logger().info(f"Response: {response.text}")
            else:
                self.get_logger().warn(f"HTTP {response.status_code}: {response.text}")
        except Exception as e:
            pass
            #self.get_logger().error(f"Failed to connect or request error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = NodeMCUClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
