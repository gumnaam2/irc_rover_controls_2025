#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import re
import threading

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.cam_publisher = self.create_publisher(String, 'bio_photo_command', 10)
        self.pid_publisher = self.create_publisher(Int32, 'bio_pid_target', 10)
        self.rel_publisher = self.create_publisher(Int32, 'bio_relay', 10)
        self.switch_publisher=self.create_publisher(Int32, 'switching_command',10)
        #self.valid_commands = ['command1', 'command2', 'command3']

        # Define the updated regex pattern
        self.COMMAND_PATTERN = (
            r'^CAM_cont$|^CAM_stop$|^CAM_test$|'
            r'^CAM_\d$|'  # Allow any positive number after "CAM_"
            r'^PID_[0-4]$|'
            r'^REL_[1-8]$|'
            r'^SW_[0-1]$'
        )

        # Compile the pattern for efficiency
        self.command_regex = re.compile(self.COMMAND_PATTERN)

        # Start the input listener thread
        #self.input_thread = threading.Thread(target=self.run, daemon=False)
        #self.input_thread.start()
        self.run()

    def is_valid_command(self, command: str) -> bool:
        """Check if the command matches the predefined patterns."""
        return bool(self.command_regex.match(command))
    def run(self):
        self.get_logger().info('The following is the key for commands-')
        self.get_logger().info('CAM_cont=start taking continuous pictures from camera')
        self.get_logger().info('CAM_stop=stop taking continuous pictures from camera')
        self.get_logger().info('CAM_test=test which cameras are working')
        self.get_logger().info('CAM_*number*=if number is 0,1,2 then takes photo from that camera')
        self.get_logger().info('CAM_*number*=if any other then sets time between clicks to that')
        self.get_logger().info('PID_*0/1/2/3/4*=Positions of PID(0=Home,1=Box 1,2= Box 2,3=Box 3,4= Driling Position)')
        self.get_logger().info('REL_*1/2/3/4/5/6/7/8*=(1=Start Heater 1,2=Start heater 2,3=Start heater 3,4=Stop 1,5=Stop 2,6=Stop 3,7=Start LED,8=Stop LED')

        while True:
            try:
                command = input('Enter a command: ')
                #self.get_logger().info('Asked for command')

                if self.is_valid_command(command):
                    if command.startswith('PID'):
                        msg = Int32()
                        msg.data = int(command[-1])
                        self.pid_publisher.publish(msg)
                        self.get_logger().info(f'Published PID command: {msg.data}')

                    elif command.startswith('CAM'):
                        msg = String()
                        if len(command) == 8:
                            # Corrected slicing from command[-4:0] to command[4:]
                            msg.data = command[-4:]
                        else:
                            msg.data = command[4:]
                        self.cam_publisher.publish(msg)
                        self.get_logger().info(f'Published CAM command: {msg.data}')

                    elif command.startswith('REL'):
                        msg = Int32()
                        msg.data = int(command[-1])
                        self.rel_publisher.publish(msg)
                        self.get_logger().info(f'Published REL command: {msg.data}')
                    elif command.startswith('SW'):
                        msg = Int32()
                        msg.data = int(command[-1])
                        self.switch_publisher.publish(msg)
                        self.get_logger().info(f'Published SW command: {msg.data}')
                else:
                    self.get_logger().warning(f'Invalid command: {command}')
                    

            except EOFError:
                # Handle end-of-file (e.g., Ctrl+D)
                self.get_logger().info('EOF received. Shutting down node.')
                rclpy.shutdown()
                break
            except Exception as e:
                self.get_logger().error(f'Error processing command: {e}')
        self.shutdown()
                
    
    def shutdown(self):
        self.get_logger().info('Shutting down now...')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down node.')
    finally:
        controller.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
