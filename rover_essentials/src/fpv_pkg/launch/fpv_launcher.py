import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node from the 'codes' package: joystick_innerrr
        Node(
            package='codes',  # The package where 'joystick_innerrr' node is located
            executable='joystick_innerrr',  # The executable name for the joystick_innerrr node
            name='joystick_in',  # The name of the node in ROS
            output='screen',  # Output to the screen
        ),
        
        # Node from the 'FPV' package: drive_fpv
        Node(
            package='FPV',  # The package where 'drive_fpv' node is located
            executable='drive_fpv',  # The executable name for the drive_fpv node
            name='cam_subscriber',  # The name of the node in ROS
            output='screen',  # Output to the screen
        ),
        
        # Node from the 'joy' package (pre-installed with ROS2): joy_node
        Node(
            package='joy',  # The joy package (pre-installed with ROS2)
            executable='joy_node',  # The joy_node is pre-installed with ROS 2
            name='joy_node',  # The name of the node in ROS
            output='screen',  # Output to the screen
        ),
    ])
