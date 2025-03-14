#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time    
from adafruit_servokit import ServoKit   
import adafruit_adg72x 
import time
import board
import busio
import lgpio
import threading

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

        self.first_pass = False
        self.n_b_press = 0

        self.ang_dir = 0
        self.lin_dir = 0
        self.cache_drop_count = 0 #drop the cache only after a long press

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_set, 10)
        self.timer_function = self.create_timer(0.02, self.execute_cmd)


    def start_drill(self):
        self.lin_state = "stop"
        self.ang_state = "CW"
        time.sleep(5)

        self.lin_state = "down"
        self.ang_state = "CW"
        time.sleep(10)

        self.lin_state = "stop"
        self.ang_state = "CW"
        time.sleep(10)

        self.lin_state = "up"
        self.ang_state = "CW"
        time.sleep(10)

        self.lin_state = "stop"
        self.ang_state = "CW"
        time.sleep(2)

        self.lin_state = "stop"
        self.ang_state = "stop"


    def joy_set(self,joyos:Joy):
        #b button -> start and stop drill 
        #a button -> drill goes down
        #y button -> drill goes up
        #x button -> open servo to collect soil 
        #rb button -> drop cache box 

        self.a_button=joyos.buttons[0]
        self.b_button=joyos.buttons[1]
        self.x_button=joyos.buttons[2]
        self.y_button=joyos.buttons[3]
        # lb_butt=joyos.buttons[4]
        self.rb_button=joyos.buttons[5] #for switching on-off augar
        self.back_button=joyos.buttons[6] #for dropping cache box after a long press
        self.start_butt=joyos.buttons[7]

        if self.a_button == 1:
            self.first_pass = True
            if self.b_button == 1:
                self.n_b_press += 1
        else:
            self.first_pass = False

        if self.first_pass:
            if self.n_b_press >= 5:
                self.n_b_press = 0
                drill_thread = threading.Thread(target=self.start_drill)
                drill_thread.daemon = True  # Ensures thread exits when the main program does
                drill_thread.start()


    def execute_cmd(self):

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
        
        
        self.get_logger().info(f"{self.first_pass}, {self.n_b_press},{self.ang_pwm}, {self.ang_dir}, {self.lin_pwm}, {self.lin_dir}")

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
        lgpio.tx_pwm(h,LIN_PWM,frequency,0)
        lgpio.gpiochip_close(h)

if __name__ == '__main__':
    main()


