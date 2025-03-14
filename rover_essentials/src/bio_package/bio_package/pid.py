#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray #where control signal
from msg_interfaces.msg import EncoderArm  # Ensure these messages are correctly defined
import time
import numpy as np
from msg_interfaces.msg import ArmEndMotion

class BioArmController(Node):
    def __init__(self):
        super().__init__('bio_arm_controller')

        self.values = [[1.3,0.0,0.0],[1.0,0.0,0.0],[0.4,0,0]] # p,i,dbio

        # Initialize PID state for each joint
        self.pid_state = [
            {'integral': 0.0, 'previous_error': 0.0, 'last_time': None},
            {'integral': 0.0, 'previous_error': 0.0, 'last_time': None},
            {'integral': 0.0, 'previous_error': 0.0, 'last_time': None}
        ]

        # Define five hard-coded target positions (example values)
        #[Base,Shoulder,Elbow]
        self.target_positions = [
            [0,0,65535], #Home
            [3785, 2833, 65535],#Drill
            [3000, 2833, 65535],#Inter1
            [3000, -17900, 65535],#Inter2
            [-400, -17900, 65535],#Inter3
            [-400, -17900, 83211],#Box1
            [-400, -17900, 40406],#Box2
            [-400, -17900, 6290]#Box3
        ]

        self.current_target = [0,0,0]  # Initialize current target
        self.current_encoders = [0, 0, 0]  # Initialize current encoder values

        #State machine variables
        self.reached=True
        self.current_state=0#Can take values in indices of target_positions
        self.target_state=0#Can take values in indices of target_positions
        self.command_state=0#Can take values in 0=Home,1=Box,2=Box,3=Box,4=Drill
        # Subscribers
        self.command_subscriber = self.create_subscription(Int32,'bio_pid_target',self.command_callback,10)
        self.encoder_subscriber = self.create_subscription(EncoderArm,'encoder_arm',self.encoder_callback,10)
        # Publisher
        self.pwm_publisher = self.create_publisher(ArmEndMotion,'arm_commands_bio_pid',10)

        self.get_logger().info('BioArmController node has been started.')
        self.timer=self.create_timer(0.01,self.bio_pid_state_machine)
    def bio_pid_state_machine(self):
        if self.command_state==0:
            if self.current_state==0:
                #Box is at home position and command for home position 
                self.reached=True
                self.target_state=self.current_state
            elif self.current_state==5:
                #Box is at one of the boxes and the command for home position
                self.target_state=4
                self.reached=False
            elif self.current_state==7:
                self.target_state=4
                self.reached=False
            elif self.current_state==6:
                self.target_state=4
                self.reached=False
            elif self.current_state==4:
                self.target_state=0
                self.reached=False
            elif self.current_state==3:
                self.target_state=0
                self.reached=False
            elif self.current_state==2:
                self.target_state=0
                self.reached=False
            elif self.current_state==1:
                self.target_state=2
                self.reached=False
        elif self.command_state==1:
            if self.current_state==0:
                self.target_state=2
                self.reached=False
            elif self.current_state==1:
                self.target_state=2
                self.reached=False
            elif self.current_state==2:
                self.target_state=3
                self.reached=False
            elif self.current_state==3:
                self.target_state=4
                self.reached=False
            elif self.current_state==4:
                self.target_state=5
                self.reached=False
            elif self.current_state==5:
                self.target_state=5
                self.reached=True
            elif self.current_state==6:
                self.target_state=5
                self.reached=False
            elif self.current_state==7:
                self.target_state=5
                self.reached=False
        elif self.command_state==2:
            if self.current_state==0:
                self.target_state=2
                self.reached=False
            elif self.current_state==1:
                self.target_state=2
                self.reached=False
            elif self.current_state==2:
                self.target_state=3
                self.reached=False
            elif self.current_state==3:
                self.target_state=4
                self.reached=False
            elif self.current_state==4:
                self.target_state=6
                self.reached=False
            elif self.current_state==5:
                self.target_state=6
                self.reached=False
            elif self.current_state==6:
                self.target_state=6
                self.reached=True
            elif self.current_state==7:
                self.target_state=6
                self.reached=True
        elif self.command_state==3:
            if self.current_state==0:
                self.target_state=2
                self.reached=False
            elif self.current_state==1:
                self.target_state=2
                self.reached=False
            elif self.current_state==2:
                self.target_state=3
                self.reached=False
            elif self.current_state==3:
                self.target_state=4
                self.reached=False
            elif self.current_state==4:
                self.target_state=7
                self.reached=False
            elif self.current_state==5:
                self.target_state=7
                self.reached=False
            elif self.current_state==6:
                self.target_state=7
                self.reached=False
            elif self.current_state==7:
                self.target_state=7
                self.reached=True
        elif self.command_state==4:
            if self.current_state==0:
                self.target_state=1
                self.reached=False
            elif self.current_state==1:
                self.target_state=1
                self.reached=True
            elif self.current_state==2:
                self.target_state=1
                self.reached=False
            elif self.current_state==3:
                self.target_state=2
                self.reached=False
            elif self.current_state==4:
                self.target_state=3
                self.reached=False
            elif self.current_state==5:
                self.target_state=4
                self.reached=False
            elif self.current_state==6:
                self.target_state=4
                self.reached=False
            elif self.current_state==7:
                self.target_state=4
                self.reached=False
        #if self.reached==False:
            #self.get_logger().info(f'Current state is:{self.current_state}')
            #self.get_logger().info(f'Target state is:{self.target_state}')
            #self.get_logger().info(f'Command state is: {self.command_state}')
         

    def command_callback(self, msg: Int32):
        cmd = msg.data
        if cmd==0:
            self.get_logger().info('Target set for Home')
            self.command_state=cmd
        elif cmd==4:
            self.get_logger().info('Target set for drill position')
            self.command_state=cmd
        elif cmd==1 or cmd==2 or cmd==3:
            self.get_logger().info(f'Target set for box {cmd}')
            self.command_state=cmd
        else:
            self.get_logger().warn(f'Invalid command received: {cmd}. Valid range is 0-{len(self.target_positions)-1}')

    def encoder_callback(self, msg: EncoderArm):
        # Update current encoder values
        #self.get_logger().info(f'Self.reached is {self.reached}')
        try:
            #self.get_logger().info('Received encoders')
            self.current_encoders[0] = msg.arm_node0[0]
            self.current_encoders[1] = msg.arm_node1[0]
            self.current_encoders[2] = msg.arm_node2[0]
        except IndexError:
            self.get_logger().error('EncoderArm message does not contain enough data.')
            return

        #Checking if all motors reached target
        if self.reached:
            return    
        #self.reached=True
        base_reached=abs(self.current_encoders[0]-self.target_positions[self.target_state][0])<50
        shoulder_reached=abs(self.current_encoders[1]-self.target_positions[self.target_state][1])<50
        elbow_reached=abs(self.current_encoders[2]-self.target_positions[self.target_state][2])<100
        self.reached=shoulder_reached and elbow_reached and base_reached

        if self.reached:
            self.get_logger().info('Reached given target')
            self.get_logger().info(f'{self.current_encoders}')
            self.current_state=self.target_state
            #Reseting all values to act against integral windup
            self.pid_state = [{'integral': 0.0, 'previous_error': 0.0, 'last_time': None},
            {'integral': 0.0, 'previous_error': 0.0, 'last_time': None},
            {'integral': 0.0, 'previous_error': 0.0, 'last_time': None}]
            pwm_msg=ArmEndMotion()
            pwm_msg.speed=[0,0,0,0,0,255]
            pwm_msg.direction=[0,0,0,0,0,0]
            #self.get_logger().info(f'Computed speed:{pwm_msg.speed}')
            self.pwm_publisher.publish(pwm_msg)
            
            return
        
        # Compute PWM values using PID controllers
        else:
            #self.get_logger().info('Calculating PID')
            pwm_values = []
            dir_values = []
            for i in range(3):
                pwm,direction = self.compute_pid(i, self.target_positions[self.target_state][i], self.current_encoders[i])
                pwm_values.append(pwm)
                dir_values.append(direction)

        # Create and publish BioArmControl message
        #pwm_msg = Float32MultiArray()
        #pwm_msg.data=pwm
            #self.get_logger().info(f'PWM computed is {pwm_values}')
            pwm_msg=ArmEndMotion()
            pwm_msg.speed=[pwm_values[0],pwm_values[1],pwm_values[2],0,0,255]
            pwm_msg.direction=[dir_values[0],dir_values[1],dir_values[2],0,0,255]
        #self.get_logger().info(f'Computed speed:{pwm_msg.speed}')
            self.pwm_publisher.publish(pwm_msg)

        #self.get_logger().debug(f'Encoders: {self.current_encoders}, Target: {self.current_target}, PWM: {pwm_values}')

    def compute_pid(self, joint_index, setpoint, measurement):
        """
        Compute the PID output for a given joint.
        """
        params = self.values[joint_index]
        state = self.pid_state[joint_index]

        current_time = time.time()
        error = setpoint - measurement

        delta_time = 0.0
        if state['last_time'] is not None:
            delta_time = current_time - state['last_time']
        state['last_time'] = current_time

        if delta_time <= 0.0:
            delta_time = 1e-16  # Prevent division by zero

        # Update integral
        state['integral'] += error * delta_time

        # Compute derivative
        derivative = (error - state['previous_error']) / delta_time

        # Compute PID output
        output = (params[0] * error) + (params[1] * state['integral']) + (params[2] * derivative)
        
        if output>0:
            direction=1
        else:
            direction=0
        if joint_index==2:
            if output>0:
                direction=0
            else:
                direction=1

        # Clamp output to [0, 255]
        output = max(0, min(150, abs(output)))

        if joint_index==2:
            self.get_logger().info(f'The base error is {error} and pwm is {output} and direction is {direction}')
        
        # Update previous error
        state['previous_error'] = error

        return int(output),direction

def main(args=None):
    rclpy.init(args=args)
    controller = BioArmController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard Interrupt (SIGINT) received. Shutting down node.')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
