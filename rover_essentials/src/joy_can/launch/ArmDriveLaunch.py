from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

package = 'joy_can'

armcan = os.path.join(get_package_share_directory(package),'launch/ArmCAN.sh')
drivecan = os.path.join(get_package_share_directory(package),'launch/DriveCAN.sh')

os.system(armcan) #Arm CAN channel
os.system(drivecan) #Drive CAN channel

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package = package,
                executable = 'drive_master',
                name = 'drive_master'
            ),
            Node(
                package=package,
                executable='arm_master',
                name='arm_master'
            ),
        ]
    )
