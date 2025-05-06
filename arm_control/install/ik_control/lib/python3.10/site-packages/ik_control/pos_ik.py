import rclpy
from rclpy.node import Node
import numpy as np
import roboticstoolbox as rtb
from sensor_msgs.msg import JointState, Joy
from msg_interfaces.msg import EncoderArm, TargetPose
import time

# Define the path to the URDF file
urdf_path = '/home/sudhindra/arm_demo/arm_model.urdf'  # Update this path
robot = rtb.ERobot.URDF(urdf_path)


class VisualizeJoints(Node):
    def __init__(self):
        super().__init__('visualize_joints')

        # Publisher for the 'joint_states' topic
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.target_pub = self.create_publisher(TargetPose, 'target_encoder', 10)
        
        # Timer to call the publish_joint_velocities function at 50 Hz
        # NOTE: timer is actually set to 0.1 -> 10 Hz, so adjust if truly 50 Hz is desired
        self.timer = self.create_timer(0.001, self.publish_joint_velocities)

        # Create subscription to joystick inputs
        self.joystick_subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10
        )

        # Initialize desired velocity and joint state
        self.desired_velocity = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        self.joint_state = JointState()
        self.joint_state.name = [f'joint_{i+1}' for i in range(len(robot.q))]
        self.joint_state.position = list(robot.q)

        # Control frequency
        self.control_frequency = 50  # Hz
        self.dt = 1.0 / self.control_frequency

        self.x_vel_cmd = 0.0
        self.y_vel_cmd = 0.0
        self.z_vel_cmd = 0.0

        self.cross_ver = 0

        self.base_target_pos = 0.0
        self.shoulder_target_pos = 0.0
        self.elbow_target_pos = 0.0
        self.wrist_up_down = 0.0
        self.wrist_rot = 0.0

        robot.q[0] = -0.13
        robot.q[1] = 0.46
        robot.q[2] = 0.32

        self.switch = False

        self.target_pose = TargetPose()

    def joystick_callback(self, joystick):
        a_butt=joystick.buttons[0]
        b_butt=joystick.buttons[1]
        x_butt=joystick.buttons[2]
        y_butt=joystick.buttons[3]
        lb_butt=joystick.buttons[4]
        rb_butt=joystick.buttons[5]
        self.cross_ver = joystick.axes[7]
        start_butt=joystick.buttons[7]

        # if cross_ver == 1:
        #     self.x_vel_cmd = 0.1
        #     self.get_logger().info(f"{cross_ver}, {self.x_vel_cmd}")
        #     time.sleep(1)
        #     self.x_vel_cmd = -0.1
        #     self.get_logger().info(f"{cross_ver}, {self.x_vel_cmd}")
        #     time.sleep(1)

        # else:
        if lb_butt == 1:
            self.x_vel_cmd = 0.006
        elif rb_butt == 1:
            self.x_vel_cmd = -0.006
        else:
            self.x_vel_cmd = 0

        if y_butt == 1:
            self.y_vel_cmd = 0.006
        elif a_butt == 1:
            self.y_vel_cmd = -0.006
        else:
            self.y_vel_cmd = 0

        if x_butt == 1:
            self.z_vel_cmd = 0.006
        elif b_butt == 1:
            self.z_vel_cmd = -0.006
        else:
            self.z_vel_cmd = 0

        self.desired_velocity[:3] = [self.y_vel_cmd, self.x_vel_cmd, self.z_vel_cmd]

    def compute_joint_velocities(self):
        """
        Computes joint velocities using the Jacobian-based inverse kinematics.
        """
        # Jacobian in the end-effector frame
        J_ee = robot.jacobe(robot.q)
        # Current end-effector pose
        T_base_to_world = robot.fkine(robot.q)
        R_base_to_world = T_base_to_world.R
        
        # Transform local (EE) Jacobian to world frame
        J_v_world = R_base_to_world @ J_ee[:3, :]  # linear part
        J_w_world = R_base_to_world @ J_ee[3:, :]  # angular part
        J_world = np.vstack((J_v_world, J_w_world))

        # Solve for joint velocities
        joint_velocities = np.linalg.pinv(J_world) @ self.desired_velocity
        return joint_velocities

    def publish_joint_velocities(self):
        joint_velocities = self.compute_joint_velocities()
        self.get_logger().info(f"published joint velocities {robot.q}")
        
        # Update robot joint angles
        robot.q += joint_velocities * self.dt

        # Convert from continuous joint angles to desired encoder positions
        # Example: for base
        self.base_target_pos = 0 # ((0.08 - robot.q[0]) * 119200) / 0.46 - 59600
        self.shoulder_target_pos = ((0.46-robot.q[1]) * 17330) / 3.14
        self.elbow_target_pos = -((0.32-robot.q[2]) * 2* 14041) / 4.71
        self.wrist_up_down = 0
        self.wrist_rot = 0
        
        # Enforce joint limits (example):
        joint_limits = [
            (-0.38,  0.08),     # Joint 1
            (-1.5708, 1.5708),  # Joint 2
            (-3.1416, 3.1416),  # Joint 3
            (-1.5708, 1.5708),  # Joint 4
        ]
        for i, (lower, upper) in enumerate(joint_limits):
            robot.q[i] = np.clip(robot.q[i], lower, upper)

        # Prepare JointState message
        self.joint_state.position = [round(q, 2) for q in robot.q]
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        # Publish JointState
        self.publisher_.publish(self.joint_state)

        # Publish TargetPose
        self.target_pose.base = float(self.base_target_pos)
        self.target_pose.shoulder = float(self.shoulder_target_pos)
        self.target_pose.elbow = float(self.elbow_target_pos)
        if self.cross_ver == 1:
            self.target_pose.shoulder = float(self.shoulder_target_pos) + 137.0
            self.target_pose.elbow = float(self.elbow_target_pos) + 151.0
            self.target_pub.publish(self.target_pose)
            time.sleep(0.1)
            self.target_pose.shoulder = float(self.shoulder_target_pos) - 137.0
            self.target_pose.elbow = float(self.elbow_target_pos) -151.0
            self.target_pub.publish(self.target_pose)
        else:
            self.target_pose.shoulder = float(self.shoulder_target_pos)
            self.target_pose.elbow = float(self.elbow_target_pos)

        self.target_pose.wrist_up_down = float(self.wrist_up_down)
        self.target_pose.wrist_rot = float(self.wrist_rot)

        self.target_pub.publish(self.target_pose)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizeJoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
