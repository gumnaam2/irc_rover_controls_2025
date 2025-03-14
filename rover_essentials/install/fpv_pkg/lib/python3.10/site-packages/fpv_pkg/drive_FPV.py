#!/usr/bin/env python
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time    
from adafruit_servokit import ServoKit   
import adafruit_adg72x 
import board
import busio


address_mux = 0x47
address_pca = 0x40

mux=adafruit_adg72x.ADG72x(board.I2C(), address_mux)
pca = ServoKit(channels=16, address=address_pca, i2c=busio.I2C(board.SCL, board.SDA))

SERVO_INDEX = [1,2,3,4,5,6,7,8]  #make right, front, left , back
fpv_angle = [90,90,90,90,90,90,90,90]


direction_back = "stop"
direction = "stop"
speed_back = 0
set_cam_back = 0     
speed = 0
set_cam = 0     
cam = "front"
index =0

def joy_set(joyos):

    global direction
    global speed
    global set_cam
    global direction_back
    global speed_back
    global set_cam_back
    global cam
    global index
    left_hor = joyos.axes[0]
    left_ver = joyos.axes[1]
    # lt = joyos.axes[2]
    right_hor = joyos.axes[3]
    right_ver = joyos.axes[4]
    # rt = joyos.axes[5]    
    # step_hor = joyos.axes[6]
    # step_ver = joyos.axes[7]  
    # a_butt=joyos.buttons[0]
    # b_butt=joyos.buttons[1]
    # x_butt=joyos.buttons[2]
    # y_butt=joyos.buttons[3]
    # lb_butt=joyos.buttons[4]
    # rb_butt=joyos.buttons[5]
    # back_butt=joyos.buttons[6]
    # start_butt=joyos.buttons[7]
    right_fpv = joyos.buttons[1]
    left_fpv = joyos.buttons[2]
    front_fpv = joyos.buttons[3]
    set_cam_back=joyos.buttons[9]
    set_cam=joyos.buttons[10]

    if left_hor > 0.0:
        direction_back = "left_rot"
        speed_back = left_hor
    elif left_hor < 0.0:
        direction_back = "right_rot"
        speed_back = abs(left_hor)
    elif left_ver > 0.0 : 
        direction_back = "up_rot" 
        speed_back = left_ver 
    elif left_ver < 0.0 : 
        direction_back = "down_rot" 
        speed_back = abs(left_ver)

    if right_hor > 0.0:
        direction = "left_rot"
        speed = right_hor
    elif right_hor < 0.0:
        direction = "right_rot"
        speed = abs(right_hor)
    elif right_ver > 0.0 : 
        direction = "up_rot" 
        speed = right_ver 
    elif right_ver < 0.0 : 
        direction = "down_rot" 
        speed = abs(right_ver)
        
    if right_fpv==1:
        cam = "right"
        index = 0
        mux.channel = 0
        print("000000000000000000000000000000000000000000000")
    elif left_fpv==1:
        cam = "left"
        index =2
        mux.channel = 2
        print("222222222222222222222222222222222222222222222")
    elif front_fpv==1:
        cam = "front"
        index =1
        mux.channel = 1
        print("11111111111111111111111111111111111111111111")

    print(speed," ",direction," ",speed_back," ",direction_back," ",cam)
    
    if direction == "up_rot" :
        fpv_angle[0+index*2] -= speed
    elif direction == "down_rot" :
        fpv_angle[0+index*2] += speed
    elif direction == "left_rot" :
        fpv_angle[1+index*2] += speed
    elif direction == "right_rot" : 
        fpv_angle[1+index*2] -= speed
    else :
        pass

    if direction_back == "up_rot" :
        fpv_angle[6] -= speed_back
    elif direction_back == "down_rot" :
        fpv_angle[6] += speed_back
    elif direction_back == "left_rot" :
        fpv_angle[7] += speed_back
    elif direction_back == "right_rot" : 
        fpv_angle[7] -= speed_back
    else :
        pass

    if(set_cam ==1):
        fpv_angle[0+index*2]=90
        fpv_angle[1+index*2]=90

    if(set_cam_back ==1):
        fpv_angle[6]=90
        fpv_angle[7]=90

    for i in range(8):
        fpv_angle[i] = max(0, min(180, fpv_angle[i]))
        pca.servo[SERVO_INDEX[i]].angle = fpv_angle[i]
        
    print("FPV angles : ", fpv_angle, "\n")

def main(args=None):
    for i in range(8):
        pca.servo[SERVO_INDEX[i]].set_pulse_width_range(500, 2500)
    rclpy.init(args=args)
    node = rclpy.create_node('cam_main')

    joy_sub = node.create_subscription(Joy, 'joy_camera', joy_set, 10)
    joy_sub

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
