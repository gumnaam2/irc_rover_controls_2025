import rclpy
from rclpy.node import Node
import numpy as np
import roboticstoolbox as rtb
from sensor_msgs.msg import JointState, Joy
from msg_interfaces.msg import EncoderArm, ShoulderJointVelocity, WristJointVelocity, ArmEndMotion, BaseMotion
import time
import math
import os
from ament_index_python.packages import get_package_share_directory

# # Define the path to the URDF file
# urdf_path = '/home/sudhindra/arm_demo/arm_model.urdf'  # Update this path

# Get the absolute path of the package
package_path = get_package_share_directory('ik_control')

# Construct the URDF file path
urdf_path = os.path.join(package_path, 'urdf', 'arm_model.urdf')
robot = rtb.ERobot.URDF(urdf_path)

toggle = False

class VelocityCommand(Node):
    def __init__(self):
        super().__init__('ik_hardware')

        self.pwm_pub = self.create_publisher(BaseMotion, '/only_base', 10)

        # create a publisher for giving out angular velocities
        self.sh_vel_pub = self.create_publisher(ShoulderJointVelocity, '/sh_ang_velocity', 10)

        self.el_vel_pub = self.create_publisher(ShoulderJointVelocity, '/el_ang_velocity', 10)

        self.wr_vel_pub = self.create_publisher(WristJointVelocity, '/wr_ang_velocity', 10)

        self.timer = self.create_timer(0.002, self.publish_ang_vel)

        self.receive_encoder = self.create_subscription(EncoderArm, '/encoder_arm', self.read_encoder, 10) 

        # create subscription to joystick inputs
        self.joystick_subscription = self.create_subscription(Joy, '/joy', self.joystick_callback, 10)

        # Initialize desired velocity and joint state
        self.desired_velocity = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        self.joint_state = JointState()
        self.joint_state.name = [f'joint_{i+1}' for i in range(len(robot.q))]
        self.joint_state.position = list(robot.q)

        self.base_motor_max_rpm = 211
        self.shoulder_motor_max_rpm = 105 #actually 117
        self.elbow_motor_max_rpm = 211 #actually 223
        self.wrist_up_down_max_rpm = 150
        self.wrist_rot_axis_max_rpm = 150

        self.base_speed = 0
        self.base_direction = 0

        self.scaling_sol = 0

        self.shoulder_ang_vel = 0
        self.elbow_ang_vel = 0
        self.base_ang_vel = 0
        self.wrist_up_down_ang_vel = 0
        self.wrist_rot_axis_ang_vel = 0

        self.x_vel_dir = 0
        self.y_vel_dir = 0
        self.z_vel_dir = 0

        # Control frequency
        self.control_frequency = 50  # Hz
        self.dt = 1.0 / self.control_frequency

    def read_encoder(self, enc):
        curr_base_enc = enc.arm_node2[0]
        shoulder_enc = enc.arm_node0[0]
        elbow_enc = enc.arm_node1[0]
        w_rmotor_enc = enc.arm_node3[0]
        w_lmotor_enc = enc.arm_node4[0] 

        robot.q[0] = 0.08 - ((curr_base_enc + 59600)/119200)*0.46
        robot.q[1] = 2.03 - (((shoulder_enc + 8650)/17330)*3.14 + 0.003)
        robot.q[2] = 1.8915 - (((elbow_enc + 4395)/9361)*3.1415 + 0.0924)
        robot.q[3] = 0.0
        robot.q[4] = 0.0
        self.get_logger().info(f"present joint positions {robot.q}, {curr_base_enc}, {shoulder_enc}, {elbow_enc}")

    def compute_joint_velocities(self):
        # Compute Jacobian in the end-effector frame
        J_ee = robot.jacobe(robot.q)
        # Get the end-effector pose in the world frame
        T_base_to_world = robot.fkine(robot.q)  # Homogeneous transformation matrix
        R_base_to_world = T_base_to_world.R     # Extract rotation matrix (3x3)  
        # Transform Jacobian to the world frame
        J_v_world = R_base_to_world @ J_ee[:3, :]  # Linear velocity part
        J_w_world = R_base_to_world @ J_ee[3:, :]  # Angular velocity part
        J_world = np.vstack((J_v_world, J_w_world))  # Combine into full Jacobian

        velocity_command = self.desired_velocity

        # Calculate joint velocities
        joint_velocities = np.linalg.pinv(J_world) @ velocity_command
        return joint_velocities
    

    def get_scaling_factor(self, joy_inp):
        if abs(joy_inp) -0.1 < 0.9/16:
            scaled_sol = 1/16
        elif abs(joy_inp) -0.1 < (2*0.9)/16:
            scaled_sol = 2/16
        elif abs(joy_inp) -0.1 < (3*0.9)/16:
            scaled_sol = 3/16
        elif abs(joy_inp) -0.1 < (4*0.9)/16:
            scaled_sol = 4/16
        elif abs(joy_inp) -0.1 < (5*0.9)/16:
            scaled_sol = 5/16
        elif abs(joy_inp) -0.1 < (6*0.9)/16:
            scaled_sol = 6/16
        elif abs(joy_inp) -0.1 < (7*0.9)/16:
            scaled_sol = 7/16
        elif abs(joy_inp) -0.1 < (8*0.9)/16:
            scaled_sol = 8/16
        elif abs(joy_inp) -0.1 < (9*0.9)/16:
            scaled_sol = 9/16
        elif abs(joy_inp) -0.1 < (10*0.9)/16:
            scaled_sol = 10/16
        elif abs(joy_inp) -0.1 < (11*0.9)/16:
            scaled_sol = 11/16
        elif abs(joy_inp) -0.1 < (12*0.9)/16:
            scaled_sol = 12/16
        elif abs(joy_inp) -0.1 < (13*0.9)/16:
            scaled_sol = 13/16
        elif abs(joy_inp) -0.1 < (14*0.9)/16:
            scaled_sol = 14/16
        elif abs(joy_inp) -0.1 < (15*0.9)/16:
            scaled_sol = 15/16
        else:
            scaled_sol = 1
        return scaled_sol
        
    
    def joystick_callback(self, joystick):
        global toggle

        left_hor = joystick.axes[0]
        left_ver = joystick.axes[1]

        right_hor = joystick.axes[3]
        right_ver = joystick.axes[4]
        
        # a_butt=joystick.buttons[0]
        # b_butt=joystick.buttons[1]
        # x_butt=joystick.buttons[2]
        # y_butt=joystick.buttons[3]
        # lb_butt=joystick.buttons[4]
        # rb_butt=joystick.buttons[5]
        start_button = joystick.buttons[7]
        back_button = joystick.buttons[6]


        if start_button:
            toggle = True
        if back_button:
            toggle = False

        
        if right_ver < 0:
            if abs(right_ver) >= 0.1:
                self.scaling_sol = self.get_scaling_factor(right_ver)
                self.y_vel_dir = 0.01
                self.x_vel_dir = 0.0
                self.z_vel_dir = 0.0


        elif right_ver > 0:
            if abs(right_ver) >= 0.1:
                self.scaling_sol = self.get_scaling_factor(right_ver)
                self.y_vel_dir = -0.01
                self.x_vel_dir = 0.0
                self.z_vel_dir = 0.0
            

        elif right_hor < 0:
            if abs(right_hor) >= 0.1:
                self.base_speed = int(((abs(right_hor) - 0.1) / 0.9)*255)
                self.base_direction = 0
                self.x_vel_dir = 0.01
                self.z_vel_dir = 0.0
                self.y_vel_dir = 0.0


        elif right_hor > 0:
            if abs(right_hor) >= 0.1:
                self.base_speed = int(((abs(right_hor) - 0.1) / 0.9)*255)
                self.base_direction = 1
                self.x_vel_dir = -0.01
                self.z_vel_dir = 0.0
                self.y_vel_dir = 0.0


        elif left_ver < 0:
            if abs(left_ver) >= 0.1:
                self.scaling_sol = self.get_scaling_factor(left_ver)
                self.z_vel_dir = 0.01
                self.x_vel_dir = 0.0
                self.y_vel_dir = 0.0
    


        elif left_ver > 0:
            if abs(left_ver) >= 0.1:
                self.scaling_sol = self.get_scaling_factor(left_ver)
                self.z_vel_dir = -0.01
                self.x_vel_dir = 0.0
                self.y_vel_dir = 0.0

        else:
            self.scaling_sol = 0
            self.x_vel_dir = 0
            self.y_vel_dir = 0
            self.z_vel_dir = 0

        base_cmd = BaseMotion()
        base_cmd.speed = self.base_speed
        base_cmd.direction = self.base_direction

        self.pwm_pub.publish(base_cmd)

        self.desired_velocity[:3] = [self.x_vel_dir, self.y_vel_dir, self.z_vel_dir]  


    def publish_ang_vel(self):
        joint_velocities = self.compute_joint_velocities()

        # elbow_ratio = joint_velocities[2]/joint_velocities[1]
        # wrist_up_down_axis = (joint_velocities[3]/joint_velocities[1])*50
        # wrist_rot_axis_ratio = (joint_velocities[4]/joint_velocities[1])*50

        # self.elbow_ang_vel = elbow_ratio * self.elbow_motor_max_rpm
        # self.wrist_rot_axis_ang_vel = wrist_rot_axis_ratio * self.wrist_rot_axis_max_rpm
        # self.wrist_up_down_ang_vel = wrist_up_down_axis * self.wrist_up_down_max_rpm

        self.shoulder_ang_vel = round((10 * self.scaling_sol * joint_velocities[1]), 4)
        self.elbow_ang_vel = round((10 * self.scaling_sol * joint_velocities[2]), 4)
        self.wrist_rot_axis_ang_vel = round(self.scaling_sol * joint_velocities[3], 3)
        self.wrist_up_down_ang_vel = round(self.scaling_sol * joint_velocities[4], 3)

        # if abs(self.elbow_ang_vel) > self.elbow_motor_max_rpm:
        #     if self.elbow_ang_vel < 0:
        #         self.elbow_ang_vel = - self.elbow_motor_max_rpm
        #     else: 
        #         self.elbow_ang_vel = self.elbow_motor_max_rpm

        #     self.shoulder_ang_vel = self.elbow_ang_vel/elbow_ratio

        # if abs(self.wrist_up_down_ang_vel) > self.wrist_up_down_max_rpm:
        #     if self.wrist_up_down_ang_vel < 0:
        #         self.wrist_up_down_ang_vel = - self.wrist_up_down_max_rpm
        #     else: 
        #         self.wrist_up_down_ang_vel = self.wrist_up_down_max_rpm


        # if abs(self.wrist_rot_axis_ang_vel) > self.wrist_rot_axis_max_rpm:
        #     if self.wrist_rot_axis_ang_vel < 0:
        #         self.wrist_rot_axis_ang_vel = - self.wrist_rot_axis_max_rpm
        #     else:
        #         self.wrist_rot_axis_ang_vel = self.wrist_rot_axis_max_rpm


        sh_joint_vel = ShoulderJointVelocity()
        el_joint_vel = ShoulderJointVelocity()
        wr_joint_vel = WristJointVelocity()

        sh_joint_vel.angular_speed = float(self.shoulder_ang_vel) if not math.isnan(self.shoulder_ang_vel) else 0.0
        #sh_joint_vel.angular_speed = 0.2
        el_joint_vel.angular_speed = float(self.elbow_ang_vel) if not math.isnan(self.elbow_ang_vel) else 0.0
        wr_joint_vel.angular_speed = [float(i) for i in [0.0, 0.0]]

        self.sh_vel_pub.publish(sh_joint_vel)
        self.el_vel_pub.publish(el_joint_vel)
        self.wr_vel_pub.publish(wr_joint_vel)

        # Debug information

        self.get_logger().info(f"Joint velocities: {self.shoulder_ang_vel}, {self.elbow_ang_vel}")
        # self.get_logger().info(f"Shoulder: {self.shoulder_ang_vel}, Elbow: {self.elbow_ang_vel}, Wrist: {self.wrist_up_down_ang_vel}, {self.wrist_rot_axis_ang_vel}")

        
def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommand()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



 