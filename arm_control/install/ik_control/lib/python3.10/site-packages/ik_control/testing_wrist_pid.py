import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from msg_interfaces.msg import ArmEndMotion, EncoderArm, ArmAngle

# Global toggle for system check
toggle = False
gripper_state = 255

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joint2_testing')

        # Publisher for arm motion commands
        self.arm_publisher = self.create_publisher(ArmEndMotion, 'arm_commands', 10)

        # Subscriber for joystick inputs
        self.joystick_subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10
        )

        self.receive_encoder = self.create_subscription(EncoderArm, '/encoder_arm', self.encoder_callback, 10) 

        # Initialize arm motion command message
        self.arm_command = ArmEndMotion()

        self.base_speed = 0
        self.base_direction = 1
        
        self.joint2_speed = 0
        self.joint2_direction = 1

        self.left_motor_speed = 0
        self.left_motor_direction = 1

        self.right_motor_speed = 0
        self.right_motor_direction = 1

        self.joint3_speed = 0
        self.joint3_direction = 1   

        self.gripper_state = 255

        self.current_base_position = 0
        self.current_shoulder_pos = 0
        self.current_elbow_pos = 0

        # Control loop frequency (50 Hz)
        self.control_frequency = 250.0
        self.dt = 1.0 / self.control_frequency

        self.wrist_l_cmd = 0
        self.wrist_r_cmd = 0

        self.wrist_l_dir = 0
        self.wrist_r_dir = 0

        self.wrist_left = 0
        self.wrist_right = 0

        self.angle_pub = self.create_publisher(ArmAngle, '/arm_angle', 10)

        # Create a timer for the control loop
        self.control_timer = self.create_timer(self.dt, self.update_wrist_cmd)

        self.wrist_receiver = self.create_subscription(ArmEndMotion, 'arm_commands_wrist_pid', self.wrist_callback, 10)

    def encoder_callback(self, msg: EncoderArm):
        """
        Callback for receiving the current encoder pulse count.
        """
        self.current_base_position = msg.arm_node2[0]
        self.current_shoulder_pos = msg.arm_node0[0]
        self.current_elbow_pos = msg.arm_node1[0]
        self.wrist_left = msg.arm_node3[0]
        self.wrist_right = msg.arm_node4[0]


    def update_wrist_cmd(self):
        self.shoulder_alpha = 1.57 - ((self.current_shoulder_pos*3.1415)/17330)
        self.elbow_beta = 3.1415 - (1.57 - ((self.current_elbow_pos*3.1415)/(2*9405)))
        angle_update = ArmAngle()
        angle_update.alpha = float(self.shoulder_alpha)
        angle_update.beta = float(self.elbow_beta)

        self.angle_pub.publish(angle_update)

    def wrist_callback(self, msg: ArmEndMotion):
        self.wrist_l_cmd = msg.speed[3]
        self.wrist_r_cmd = msg.speed[4]

        self.wrist_l_dir = msg.direction[3]
        self.wrist_r_dir = msg.direction[4]


    def joystick_callback(self, joystick):
        global toggle
        global gripper_state

        a_butt = joystick.buttons[0]
        b_butt = joystick.buttons[1]
        x_butt = joystick.buttons[2]
        y_butt = joystick.buttons[3]

        lt_butt = joystick.axes[2]
        rt_butt = joystick.axes[5]

        cross_ver = joystick.axes[7]

        right_hor = joystick.axes[3]
        right_ver = joystick.axes[4]

        start_button = joystick.buttons[7]
        back_button = joystick.buttons[6]

        rb_butt=joystick.buttons[5]


        if cross_ver == 1:
            self.gripper_state = 0

        if cross_ver == -1:
            self.gripper_state = 255

        if rb_butt == 1:
            self.arm_command.reset = True
        else:
            self.arm_command.reset = False

        # if x_butt == 1:
        #     self.left_motor_speed = 120
        #     self.left_motor_direction = 1

        #     self.right_motor_speed = 120
        #     self.right_motor_direction = 0


        # elif b_butt == 1:
        #     self.left_motor_speed = 120
        #     self.left_motor_direction = 0

        #     self.right_motor_speed = 120
        #     self.right_motor_direction = 1

        # elif y_butt == 1:
        #     self.left_motor_speed = 120
        #     self.left_motor_direction = 1

        #     self.right_motor_speed = 120
        #     self.right_motor_direction = 1

        # elif a_butt == 1:
        #     self.left_motor_speed = 120
        #     self.left_motor_direction = 0

        #     self.right_motor_speed = 120
        #     self.right_motor_direction = 0  
        
        # else:
        #     self.left_motor_direction = 0
        #     self.left_motor_speed = 0
        #     self.right_motor_speed = 0
        #     self.right_motor_direction = 0

        if lt_butt < 1.0:
            if lt_butt >= 0.9:
                self.joint3_speed = 0
                self.joint3_direction = 0
            else: 
                self.joint3_speed = int(((0.9 - lt_butt) / 1.9)*200)
                self.joint3_direction = 0

        elif rt_butt < 1.0:
            if rt_butt >= 0.9:
                self.joint3_speed = 0
                self.joint3_direction = 1
            else: 
                self.joint3_speed = int(((0.9 - rt_butt) / 1.9)*200)
                self.joint3_direction = 1
        
        else:
            self.joint3_direction = 0
            self.joint3_speed = 0


        if right_ver < 0:  
            if abs(right_ver) <= 0.1:
                self.joint2_speed = 0
                self.joint2_direction = 0
            else: 
                self.joint2_speed = int(((abs(right_ver) - 0.1) / 0.9)*255)
                self.joint2_direction = 0

        elif right_ver > 0:  
            if right_ver <= 0.1:
                self.joint2_speed = 0
                self.joint2_direction = 1

            else:
                self.joint2_speed = int(((right_ver - 0.1) / 0.9)*255) 
                self.joint2_direction = 1  
        
        else:
            self.joint2_direction = 0
            self.joint2_speed = 0

        # elif right_hor < 0:  
        #     if self.current_base_position >= 5000 and self.current_base_position <= 114200:
        #         if abs(right_hor) <= 0.1:
        #             self.base_speed = 0
        #             self.base_direction = 1
        #         else: 
        #             self.base_speed = int(((abs(right_hor) - 0.1) / 0.9)*255)
        #             self.base_direction = 1
        #     else:
        #             self.base_speed = 0
        #             self.base_direction = 1


        # elif right_hor > 0:  
        #     if self.current_base_position >= 5000 and self.current_base_position <= 114200:
        #         if right_hor <= 0.1:
        #             self.base_speed = 0
        #             self.base_direction = 0

        #         else:
        #             self.base_speed = int(((right_hor - 0.1) / 0.9)*255) 
        #             self.base_direction = 0  
        #     else:
        #             self.base_speed = 0
        #             self.base_direction = 0

        if right_hor < 0:  
            if abs(right_hor) <= 0.1:
                self.base_speed = 0
                self.base_direction = 1
            else: 
                self.base_speed = int(((abs(right_hor) - 0.1) / 0.9)*255)
                self.base_direction = 1


        elif right_hor > 0:  
            if right_hor <= 0.1:
                self.base_speed = 0
                self.base_direction = 0

            else:
                self.base_speed = int(((right_hor - 0.1) / 0.9)*255) 
                self.base_direction = 0 
        else:
            self.base_direction = 0
            self.base_speed = 0 


        # else:
        #     self.base_speed = 0
        #     self.base_direction = 1

        #     self.joint2_speed = 0
        #     self.joint2_direction = 1

        #     self.left_motor_speed = 0
        #     self.left_motor_direction = 1

        #     self.right_motor_speed = 0
        #     self.right_motor_direction = 1

        #     self.joint3_speed = 0
        #     self.joint3_direction = 1   

        if start_button:
            toggle = True
        if back_button:
            toggle = False

        # if self.wrist_left > 15000 and self.wrist_l_dir == 0:
        #     self.wrist_l_cmd = 0
        # if self.wrist_right < -15000 and self.wrist_r_dir == 1:
        #     self.wrist_r_cmd = 1

        self.arm_command.sys_check = toggle
        self.arm_command.speed = [self.joint2_speed, self.joint3_speed, self.base_speed, self.wrist_l_cmd, self.wrist_r_cmd, self.gripper_state] # change this according to position corresponding to address of joint2
        self.arm_command.direction = [self.joint2_direction, self.joint3_direction, self.base_direction, self.wrist_l_dir, self.wrist_r_dir, 0]

        # self.arm_command.speed = [self.joint2_speed, self.joint3_speed, self.base_speed, 0, 0, self.gripper_state] # change this according to position corresponding to address of joint2
        # self.arm_command.direction = [self.joint2_direction, self.joint3_direction, self.base_direction, 0, 0, 0]

        # Debug information
        self.get_logger().info(f"Sending command to hardware: PWM {self.arm_command.speed}, DIR {self.arm_command.direction}, Syscheck {self.arm_command.sys_check}, Reset {self.arm_command.reset}") 

        self.arm_publisher.publish(self.arm_command)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
                   

'''
pin=18
freq=50

#Setting PWM according to the Servo motor chosen
h = lgpio.gpiochip_open(0)

#Turning the rover by the angle necessary
def Turn(angle_to_point):
    duty = ((angle_to_point/90)*5)+7.5
    print(duty)
    if duty<2.5:
        duty=2.5
    lgpio.tx_pwm(h, pin, freq, duty)
    sleep(0.1)
    return

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_testing')

        #  Publisher for arm motion commands
        self.gripper_cmd_subscriber = self.create_subscription(ArmEndMotion, 'arm_commands', self.jo>

        # Initialize arm motion command message
        self.received_arm_command = ArmEndMotion()

    def joystick_callback(self, cmd):
        #get the last message of arm commands
        val = cmd.speed[-1]

        if val == 0:
            Turn(-90)
        if val == 255:
            Turn(45)


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
'''

