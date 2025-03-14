import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import os
import time
from threading import Thread

#TODO
#Check if 3 cameras can work through USB hub

class PhotoCaptureNode(Node):
    def __init__(self):
        super().__init__('photo_capture_node')

        # Parameters
        self.declare_parameter('save_directory0', 'photos0')
        self.declare_parameter('save_directory1', 'photos1')
        self.declare_parameter('save_directory2', 'photos2')
        self.declare_parameter('capture_time', 1.0)  # in Hz
        self.save_directory=[]
        self.save_directory.append(self.get_parameter('save_directory0').get_parameter_value().string_value)
        self.save_directory.append(self.get_parameter('save_directory1').get_parameter_value().string_value)
        self.save_directory.append(self.get_parameter('save_directory2').get_parameter_value().string_value)
        self.capture_time = self.get_parameter('capture_time').get_parameter_value().double_value

        # Create directory if it doesn't exist
        for i in range(3):
            if not os.path.exists(self.save_directory[i]):
                os.makedirs(self.save_directory[i])

        # Subscriber
        self.command_subscriber = self.create_subscription(
            String, 'bio_photo_command', self.command_callback, 10)

        # Start continuous photo capture thread
        self.continuous_capture = False
        self.continuous_thread = Thread(target=self.call_cont)
        self.continuous_thread.daemon = True
        self.continuous_thread.start()
        #if not self.continous_capture:
        #    self.timer=self.create_timer(1.0,self.continous_photo_capture)
        # Test cameras
        self.test_cameras()
    
    def test_cameras(self):
        self.cameras = []
        for i in range(3):  # Assuming 3 USB webcams
            try:
                cap = cv2.VideoCapture(i)
            except:
                self.get_logger().error(f'Failed to initialize camera {i}.')
            if cap.isOpened():
                self.get_logger().info(f"Camera {i} working")
                self.cameras.append(i)
            else:
                self.get_logger().warning(f"Failed to initialize camera {i}.")
            cap.release()
    def command_callback(self, msg):
        command = msg.data
        if command == 'cont':
            self.continuous_capture=True
            self.get_logger().info('Taking continous photos')
            #self.continuous_photo_capture()
        elif command == 'stop':
            self.continuous_capture=False
            self.get_logger().info('Stopping continous photos')
            #self.continuous_photo_capture()
        elif command =='test':
            self.test_cameras()
        elif command.isdigit():
            if int(command)<3:
                self.take_photos(int(command))
            elif int(command)>2:
                self.capture_time=int(command)
                self.get_logger().info(f'Updated continous photo taking time to {self.capture_time}secs')
        else:
            self.get_logger().warning(f"Unknown command received: {command}")

    def take_photos(self,cam_id=-1):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        #for cam_id, cap in self.cameras:
        if cam_id!=-1:
            cap=cv2.VideoCapture(cam_id)
            ret, frame = cap.read()
            if ret:
                filename = os.path.join(self.save_directory[cam_id], f'camera_{cam_id}_{timestamp}.jpg')
                cv2.imwrite(filename, frame)
                self.get_logger().info(f"Photo saved: {filename}")
            else:
                self.get_logger().error(f"Failed to capture photo from camera {cam_id}.")
            cap.release()
            return
        for cam_id in self.cameras:
            cap=cv2.VideoCapture(cam_id)
            ret, frame = cap.read()
            if ret:
                filename = os.path.join(self.save_directory[cam_id], f'camera_{cam_id}_{timestamp}.jpg')
                cv2.imwrite(filename, frame)
                self.get_logger().info(f"Photo saved: {filename}")
            else:
                self.get_logger().error(f"Failed to capture photo from camera {cam_id}.")
            cap.release()
    def call_cont(self):
        #self.get_logger().info('Calling cont always')
        while True:
            self.continuous_photo_capture()
        return

    def continuous_photo_capture(self):
        while self.continuous_capture:
            self.take_photos()
            time.sleep(self.capture_time)

    def destroy_node(self):
        # Clean up
        self.continuous_capture = False
        self.continuous_thread.join()
        #for cam_id, cap in self.cameras:
        #    cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    photo_capture_node = PhotoCaptureNode()

    try:
        rclpy.spin(photo_capture_node)
    except KeyboardInterrupt:
        pass
    finally:
        photo_capture_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

