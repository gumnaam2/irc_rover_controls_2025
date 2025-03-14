import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import threading
import traceback
from time import sleep
from adafruit_servokit import ServoKit
from stitching import Stitcher
# pip install adafruit-servokit stitching opencv-python

# Global variables for the panorama
panorama_ready = False
imgs = []

# Setting up the ServoKit
kit = ServoKit(channels=16)
pin = 9  # Adjust to the correct channel for your servo (typically 0-15)
freq = 50

# Camera capture function
def capture(cam):
    global imgs
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame")
        return
    frame = cv2.resize(frame, (0, 0), fx=0.9, fy=0.9)
    imgs.append(frame)
    return

# Setup camera to capture frames
def setup_camera(frame, wait_time):
    global imgs, panorama_ready
    cam = cv2.VideoCapture('/dev/video0')  # Adjust to your camera device
    print("Capturing Panorama")
    for i in range(frame):
        capture(cam)
        sleep(wait_time)
    print(f"Captured {len(imgs)} frames")
    panorama_ready = True
    stitcher = Stitcher(detector="sift", confidence_threshold=0.9)
    print("Now processing panorama")
    panorama = stitcher.stitch(imgs)
    cv2.imshow('Final Panorama', panorama)
    cv2.imwrite('panorama.jpg', panorama)
    cv2.waitKey(0)
    cam.release()
    cv2.destroyAllWindows()

# Servo movement function
def turn_servo():
    global panorama_ready
    try:
        i = 0
        while True:
            angle_to_point = (15 * i - 90)
            i += 1
            if i == 13:
                i = 0
            duty = ((angle_to_point / 90) * 5) + 7.5
            if duty < 2.5:
                duty = 2.5
            kit.servo[pin].angle = angle_to_point
            sleep(0.2)
            if duty == 2.5:
                sleep(0.8)
            if panorama_ready:
                print("Images captured...Stopping Servo")
                break
    except (KeyboardInterrupt, SystemExit):
        print('Exit due to keyboard interrupt')
    except Exception as ex:
        print('Python error with no exception handler:')
        print('Traceback error:', ex)
        traceback.print_exc()
    finally:
        return

# ROS2 Node to control panorama capture and servo
class Panorama(Node):
    def __init__(self, frames, wait_time, is_servo):
        super().__init__('panorama_node')
        self.num_frames = frames
        self.wait_time = wait_time
        self.is_servo = is_servo
        self.panorama_ready_pub = self.create_publisher(String, 'panorama_ready', 10)
        
        if self.is_servo:
            self.get_logger().info("Starting servo")
            self.servo_thread = threading.Thread(target=self.run_servo)
        else:
            self.get_logger().info("Starting camera capture")
            self.camera_thread = threading.Thread(target=self.run_camera)
        
    def run_servo(self):
        turn_servo()

    def run_camera(self):
        setup_camera(self.num_frames, self.wait_time)
        panorama_ready_msg = String()
        panorama_ready_msg.data = 'Panorama is ready'
        self.panorama_ready_pub.publish(panorama_ready_msg)
        self.get_logger().info('Published Panorama Ready message')

    def start(self):
        if self.is_servo:
            self.servo_thread.start()
        else:
            self.camera_thread.start()

def main(args=None):
    rclpy.init(args=args)
    
    # Start the servo and camera threads
    panorama_servo = Panorama(50, 0.1, True)
    panorama_camera = Panorama(50, 0.1, False)
    
    panorama_servo.start()
    panorama_camera.start()
    
    rclpy.spin(panorama_servo)
    rclpy.spin(panorama_camera)
    
    panorama_servo.servo_thread.join()
    panorama_camera.camera_thread.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

