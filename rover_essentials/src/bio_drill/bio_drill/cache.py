import rclpy
from rclpy.node import Node
import requests
from sensor_msgs.msg import Joy

class NodeMCUClient(Node):

    def __init__(self):
        super().__init__('node_mcu_client')

        self.declare_parameter('nodemcu_ip','192.168.69.95')
        self.declare_parameter('command','open')
        self.declare_parameter('update_rate',1.0)

        self.nodemcu_ip = self.get_parameter('nodemcu_ip').value
        self.command = self.get_parameter('command').value
        self.update_rate = self.get_parameter('update_rate').value

        self.timer = self.create_timer(self.update_rate,self.timer_callback)
        self.joystick_sub = self.create_subscription(Joy,'/joy',self.joystick_callback,10)
        self.get_logger().info(f"NodeMCU started: IP {self.nodemcu_ip} Command: {self.command}, Rate: {self.update_rate}s")

    def joystick_callback(self,joystick):
        if joystick.axes[7]==1.0:
            self.command="close"
        elif joystick.axes[7]==-1.0:
            self.command="open"
    
    def timer_callback(self):
        url = f'http://{self.nodemcu_ip}/{self.command}'
        self.get_logger().info(f"Sending request to: {url}")

        try:
            response = requests.get(url,timeout=2.0)
            if response.status_code == 200:
                self.get_logger().info(f"Response: {response.text}")
            else:
                self.get_logger().warn(f"HTTP {response.status_code}: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect or request error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = NodeMCUClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()