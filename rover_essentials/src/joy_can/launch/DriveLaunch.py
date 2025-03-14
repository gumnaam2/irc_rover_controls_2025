from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory


package = 'joy_can'

resource_path = os.path.join(get_package_share_directory(package),'launch/DriveCAN.sh')

os.system(resource_path) #run the shell file to set up Drive CAN channel

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package=package,
                executable='drive_master',
                name='drive_master'
            ),
        ]
    )
