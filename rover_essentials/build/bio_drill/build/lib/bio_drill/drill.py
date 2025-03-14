#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time    
from adafruit_servokit import ServoKit   
import adafruit_adg72x 
import board
import busio
import lgpio


address = 0x40
LIN_PWM = 5 #set the pin to control the drill relay
LIN_DIR = 6 #set the direction pin for the vertical base
#pins 29 and 31 when numbered

AUG_PWM = 13
AUG_DIR = 19

mux=adafruit_adg72x.ADG72x(board.I2C(), address)
pca = ServoKit(channels=16, address=address, i2c=busio.I2C(board.SCL, board.SDA))


cache_channel  =  7 #set the PCA channel for the cache box drop
update_rate = 0.005
frequency = 10000

h  = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h,LIN_PWM)
lgpio.gpio_claim_output(h,LIN_DIR)
lgpio.gpio_claim_output(h,AUG_PWM)
lgpio.gpio_claim_output(h,AUG_DIR)

toggle = False

class BioDrill(Node):
    def __init__(self):
        super().__init__('DrillNode')
        self.a_button = 0
        self.b_button = 0
        self.x_button = 0
        self.y_button = 0
        self.rb_button = 0
        self.start_butt = 0
        self.back_button = 0
        
        self.lin_state = "stop"
        self.ang_state = "stop"

        self.ang_pwm = 0
        self.lin_pwm = 0

        self.ang_dir = 0
        self.lin_dir = 0
        self.cache_drop_count = 0 #drop the cache only after a long press

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_set, 10)
    
    def joy_set(self,joyos:Joy):
        #b button -> start and stop drill 
        #a button -> drill goes down
        #y button -> drill goes up
        #x button -> open servo to collect soil 
        #rb button -> drop cache box 

        # left_hor = joyos.axes[0]
        # left_ver = joyos.axes[1]
        # lt = joyos.axes[2]
        # right_hor = joyos.axes[3]
        # right_ver = joyos.axes[4]
        # rt = joyos.axes[5]    
        # step_hor = joyos.axes[6]
        # step_ver = joyos.axes[7]  
        self.a_button=joyos.buttons[0]
        self.b_button=joyos.buttons[1]
        self.x_button=joyos.buttons[2]
        self.y_button=joyos.buttons[3]
        # lb_butt=joyos.buttons[4]
        self.rb_button=joyos.buttons[5] #for switching on-off augar
        self.back_button=joyos.buttons[6] #for dropping cache box after a long press
        self.start_butt=joyos.buttons[7]
        # right_fpv = joyos.buttons[1]
        # left_fpv = joyos.buttons[2]
        # front_fpv = joyos.buttons[3]
        # set_cam=joyos.buttons[10]


        if self.start_butt == 1:
            self.ang_state = "stop"
        elif self.b_button == 1:
            self.ang_state = "CW"
        elif self.x_button == 1:
            self.ang_state = "CCW"

        if self.rb_button == 1:
            self.lin_state = "stop"
        elif self.y_button == 1:
            self.lin_state = "up"
        elif self.a_button == 1:
            self.lin_state = "down"

        if self.ang_state == "stop":
            self.ang_pwm = 0
            self.ang_dir = 0
        if self.ang_state == "CW":
            self.ang_pwm = 255
            self.ang_dir = 0
        if self.ang_state == "CCW":
            self.ang_pwm = 255
            self.ang_dir = 1

        if self.lin_state == "stop":
            self.lin_pwm = 0
            self.lin_dir = 0
        if self.lin_state == "up":
            self.lin_pwm = 255
            self.lin_dir = 0
        if self.lin_state == "down":
            self.lin_pwm = 255
            self.lin_dir = 1

        if self.back_button == 1:
            self.cache_drop_count +=1
        
        if self.cache_drop_count >100:
            self.cache_drop_count = 0
        
        if self.cache_drop_count>=95:
            self.get_logger().info(f"Dropped cache box")
            # pca.servo[cache_channel].angle = 90 #dropped cache box
        elif self.cache_drop_count >=50 and self.cache_drop_count<=55:
            self.get_logger().info(f"Preparing to drop the cache box. Press a little longer")
        
        if self.cache_drop_count<95:
            pass
            # pca.servo[cache_channel].angle = 0 #do not drop the box
        
        
        self.get_logger().info(f"{self.ang_pwm}, {self.ang_dir}, {self.lin_pwm}, {self.lin_dir}")

        if self.lin_pwm == 0:
            lgpio.gpio_write(h,LIN_PWM,0) #shut off linear base
            pca.servo[cache_channel].angle = 0 #do not drop the box
        
            #pca.servo[relay_channel].angle  = 0x000
            #lgpio.tx_pwm(h,RELAY,frequency,0)
        elif self.lin_pwm == 255:
            lgpio.gpio_write(h,LIN_PWM,1)
            pca.servo[cache_channel].angle = 180 #do not drop the box
        
            #pca.servo[relay_channel].duty_cycle = 0xfff
            #lgpio.tx_pwm(h,RELAY,frequency,100)
        if self.lin_dir == 0: #turn on linear base
            lgpio.gpio_write(h,LIN_DIR,0) #move in down direction
        elif self.lin_dir == 1: #turn on linear base
            lgpio.gpio_write(h,LIN_DIR,1) #move in other direction
            
        if self.ang_pwm == 255:
            lgpio.gpio_write(h,AUG_PWM,1) #turn on the motor 
            if self.ang_dir == 0:
                lgpio.gpio_write(h,AUG_DIR,0)
            elif self.ang_dir == 1:
                lgpio.gpio_write(h,AUG_DIR,1)
        elif self.ang_pwm == 0:
            lgpio.gpio_write(h,AUG_PWM,0) #turn off the motor

    
def main(args=None):
    pca.servo[cache_channel].set_pulse_width_range(500,2500)
    rclpy.init(args=args)
    drill = BioDrill()

    #TODO subscribe to Joy node
    #TODO add control for the servo dropping the cache 
    #decide on the topic it will subscribe to here
    
    try:
        rclpy.spin(drill)
    finally:
        drill.destroy_node()
        rclpy.shutdown()
        lgpio.tx_pwm(h,IN3,frequency,0)
        lgpio.gpiochip_close(h)

if __name__ == '__main__':
    main()