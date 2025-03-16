from math import pi
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from msg_interfaces.msg import ArmAngle, EncoderArm, ArmEndMotion
import matplotlib.pyplot as plt

current_encoder_list_l = []
current_encoder_list_r = []
target_encoder_list_l = []
target_encoder_list_r = []
index = []

class WristControlNode(Node):
    def __init__(self):
        super().__init__('wrist_ik_pid')
        self.control_rate = 100
        self.wrist_publisher = self.create_publisher(ArmEndMotion, 'arm_commands_wrist_pid', 10)
        self.angle_subscription = self.create_subscription(ArmAngle, '/arm_angle', self.arm_angle_callback, 10)
        self.receive_encoder = self.create_subscription(EncoderArm, '/encoder_arm', self.encoder_callback, 10) 
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.alpha = 90
        self.prev_alpha = 90
        self.del_alpha = 0
        self.beta = 90
        self.prev_beta = 90
        self.del_beta = 0
        self.threshold = 500
        self.encoder_minus_90_a = 20400# measure one time
        self.encoder_plus_90_a =  -19664 # measure one time
        self.encoder_minus_90_b = 20400 # measure one time
        self.encoder_plus_90_b =  -19664 # measure one time
        self.encoder_per_degree_increment_a = (self.encoder_plus_90_a - self.encoder_minus_90_a)/180 
        self.encoder_per_degree_increment_b = (self.encoder_plus_90_b - self.encoder_minus_90_b)/180 
        self.last_error_a = 0.0
        self.last_error_b = 0.0
        self.integral_a = []
        self.integral_b = []
        self.target_encoder_b = 0
        self.target_encoder_a = 0
        self.current_encoder_a  = 0
        self.current_encoder_b = 0
        self.kp_a = 1
        self.ki_a = 0
        self.kd_a = 0
        self.kp_b = 10 
        self.ki_b = 0
        self.kd_b = 0
        self.last_time = None
        self.error_a = 0
        self.error_b = 0
        self.speed_a = 0
        self.speed_b = 0

    def encoder_callback(self, msg: EncoderArm):
        global current_encoder_list_l, current_encoder_list_r, index
        # Callback for receiving the current encoder pulse count.
        self.current_encoder_a = msg.arm_node3[0]
        self.current_encoder_b = msg.arm_node4[0]

        current_encoder_list_l.append(self.current_encoder_a)
        current_encoder_list_r.append(self.current_encoder_b)
        index.append(1)

    def arm_angle_callback(self, msg: ArmAngle):
        global target_encoder_list_l, target_encoder_list_r
        # self.alpha = (msg.alpha/pi)*180
        # self.beta = (msg.beta/pi)*180
        # self.del_alpha = self.alpha-self.prev_alpha
        # self.del_beta = self.beta-self.prev_beta
        # self.del_alpha /= 2
        # self.del_beta /= 2
        self.alpha=msg.alpha
        self.beta=msg.beta
        # self.target_encoder_a = self.current_encoder_a + self.encoder_per_degree_increment_a*(self.del_alpha + self.del_beta) # control variable target
        # self.target_encoder_b = self.current_encoder_b + self.encoder_per_degree_increment_b*(self.del_alpha + self.del_beta)
        self.target_encoder_a = (3.1415-(self.alpha+self.beta))*(-40064/3.1415)
        self.target_encoder_b = (3.1415-(self.alpha+self.beta))*(-40064/3.1415)
        

        target_encoder_list_l.append(self.target_encoder_a)
        target_encoder_list_r.append(self.target_encoder_b)

        # self.get_logger().info(f"curr alpha: {self.alpha}, curr_beta: {self.beta}")
        # self.get_logger().info(f"prev alpha: {self.prev_alpha}, prev_beta: {self.prev_beta}")
        # self.get_logger().info(f"del alpha: {self.del_alpha}, del_beta: {self.del_beta}")
        self.get_logger().info(f"Gamma = {3.1415 - self.alpha - self.beta}, target_encoder {self.target_encoder_a}, error: {self.error_a, self.error_b}")

    def control_loop(self):
        # Main PID control loop.



        # Calculate time delta
        # current_time = time.time()
        # dt = current_time - self.last_time if self.last_time else 1.0 / self.control_rate
        # self.last_time = current_time

        # # Calculate PID error terms
        self.error_a = self.target_encoder_a - self.current_encoder_a
        # self.integral_a.append(self.error_a * dt)
        self.error_b = self.target_encoder_b - self.current_encoder_b
        # self.integral_b.append(self.error_b * dt)

        self.control_effort_a = self.kp_a * self.error_a
        self.control_effort_b = self.kp_b * self.error_b
     
        # if len(self.integral_a)>500:
        #     deleted_value=self.integral_a.pop(0)
        # if len(self.integral_b)>500:
        #     deleted_value=self.integral_b.pop(0)

        # integral_a = np.sum(np.array(self.integral_a))
        # derivative_a = (self.error_a - self.last_self.error_a) / dt if dt > 0 else 0.0
        # self.last_self.error_a = self.error_a
        # integral_b = np.sum(np.array(self.integral_b))
        # derivative_b = (self.error_b - self.last_self.error_b) / dt if dt > 0 else 0.0
        # self.last_self.error_b = self.error_b

        abs_error_a = abs(self.error_a)
        abs_error_b = abs(self.error_b)

        avg_abs = (abs_error_a+abs_error_b)/2

        # # Compute control effort (velocity command)
        # control_effort_a = (self.kp_a * self.error_a) + (self.ki_a * integral_a) + (self.kd_a * derivative_a)
        if (self.error_a<0) and (self.error_b<0):
            reverse_a=1
            reverse_b=1
        elif (self.error_a>=0) and (self.error_b>=0):
            reverse_a=0
            reverse_b=0
        elif (self.error_a<0) and (self.error_b>=0):
            reverse_a=0
            reverse_b=1
        else:
            reverse_a=1
            reverse_b=0


        # if (error_a > 300):
        #     control_effort_a = 200
        # elif abs(control_effort_a)>200:
        #     control_effort_a=200
        # else:
        #     control_effort_a=abs(int(control_effort_a))


        # control_effort_b = (self.kp_b * self.error_b) + (self.ki_b * integral_b) + (self.kd_b * derivative_b)
        # if self.error_b<0:
        #     reverse_b=1
        # else:
        #     reverse_b=0

        # if (error_b > 300):
        #     control_effort_b = 200
        # elif abs(control_effort_b)>200:
        #     control_effort_b=200
        # else:
        #     control_effort_b=abs(int(control_effort_b))

        # # Publish control effort

        # if (error_a < self.threshold):
        #     self.prev_alpha = self.alpha
        # if (error_b < self.threshold):
        #     self.prev_beta = self.beta

        if avg_abs < self.threshold:
            self.speed_a = 0
            self.speed_b = 0
        else:
            self.speed_a = 100
            self.speed_b = 100
 
        control_msg = ArmEndMotion()
        control_msg.speed = [0,0,0,self.speed_a,self.speed_b,0]
        control_msg.direction = [0,0,0,reverse_a,reverse_b,0]
        self.wrist_publisher.publish(control_msg)    
            
            
    
    '''
    # fixed variables (DONE)
    encoder_minus_90 = # measure one time
    encoder_plus_90 =  # measure one time
    encoder_per_degree_increment = (encoder_plus_90 - encoder_minus_90)/180 
    # in a time loop: (kar raha hu)
    alpha, beta = #subscribing
    desired_angle = alpha+beta # in degrees
    target_encoder_a = encoder_minus_90 + encoder_per_degree_increment*(270 - desired_angle) # control variable target
    target_encoder_b = encoder_minus_90 + encoder_per_degree_increment*(270 - desired_angle)
    real_encoder_a = # subscribing
    real_encoder_b = # subscribing
    err_a = target_encoder_a-real_encoder_a
    err_b = target_encoder_b-real_encoder_b
    pos_pid_a(error_a):
        # publish speed+dir on arm_commands
    pos_pid_b(error_b):
        # publish  speed+dir on arm_commands
    '''

def main(args=None):
    rclpy.init(args=args)
    node = WristControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.plot(index, target_encoder_list_l, label="Target left")
        plt.plot(index, current_encoder_list_l, label="Actual left")
        plt.legend()
        plt.show()

        plt.plot(index, target_encoder_list_r, label="Target right")
        plt.plot(index, current_encoder_list_r, label="Actual right")
        plt.legend()
        plt.show()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

'''
pos_pid(a):
   feed encoder error in motor a as motor a speed
pos_pid(b):
   feed encoder error in motor b as motor b speed
'''
