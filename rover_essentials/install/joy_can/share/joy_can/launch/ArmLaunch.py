from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

package = 'joy_can'

resource_path = os.path.join(get_package_share_directory(package),'launch/ArmCAN.sh')

os.system(resource_path) #run the shell script to run Arm CAN channel

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package=package,
                executable='arm_master',
                name='arm_master'
            ),
        ]
    )
