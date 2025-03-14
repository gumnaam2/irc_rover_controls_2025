import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from geometry_msgs.msg import Point

class PanoramaOverlayNode(Node):
    def __init__(self):
        super().__init__('panorama_overlay_node')

        # Initialize CvBridge to convert ROS image messages to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to topics
        self.create_subscription(Point, 'latitude_longitude', self.lat_lon_callback, 10)
        self.create_subscription(String, 'direction', self.direction_callback, 10)
        #self.create_subscription(Image, 'panorama_image', self.image_callback, 10)

        # Variables to hold the received data
        self.latitude = None
        self.longitude = None
        self.direction = None
        self.panorama_image = cv2.imread('final_panorama.jpg',cv2.IMREAD_UNCHANGED)

        # Load compass image for direction indicator
        self.compass_image = cv2.imread('compass.png', cv2.IMREAD_UNCHANGED)  # Assumes compass.png exists
        if self.compass_image is None:
            self.get_logger().warn("Compass image not found!")

    def lat_lon_callback(self, msg):
        """Callback for latitude and longitude information"""
        self.latitude = msg.x  # Assuming msg.x is latitude, msg.y is longitude
        self.longitude = msg.y

    def direction_callback(self, msg):
        """Callback for direction information"""
        try:
            self.direction = float(msg.data)  # Assuming the direction is a float in degrees
        except ValueError:
            self.get_logger().warn("Received invalid direction data.")

    def image_callback(self, msg):
        """Callback for the panorama image"""
        # Convert the ROS Image message to OpenCV format
        self.panorama_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the panorama image
        if self.panorama_image is not None:
            self.process_image()

    def process_image(self):
        """Process the panorama image and overlay text and compass"""
        # Add the latitude, longitude, and height-to-width ratio as text
        if self.latitude is not None and self.longitude is not None:
            cv2.putText(self.panorama_image, f"Lat: {self.latitude:.6f}, Lon: {self.longitude:.6f}",
                        (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Calculate the height-to-width ratio and add it as text
        h, w, _ = self.panorama_image.shape
        height_width_ratio = h / w
        cv2.putText(self.panorama_image, f"Height/Width Ratio: {height_width_ratio:.2f}",
                    (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # If direction is available, add compass image rotated to the correct direction
        if self.direction is not None:
            self.overlay_compass(self.direction)

        # Save the modified image
        cv2.imwrite('output_panorama_with_overlay.jpg', self.panorama_image)
        self.get_logger().info("Panorama image saved as 'output_panorama_with_overlay.jpg'.")

    def overlay_compass(self, direction):
        """Overlay a compass image on the corner of the panorama, rotated to the direction"""
        compass_resized = self.resize_compass()

        # Calculate the center position for placing the compass image on the panorama
        height, width, _ = self.panorama_image.shape
        compass_x = width - compass_resized.shape[1] - 30
        compass_y = height - compass_resized.shape[0] - 30

        # Rotate the compass image based on the direction
        rotated_compass = self.rotate_image(compass_resized, direction)

        # Overlay the rotated compass image onto the panorama
        y1, y2 = compass_y, compass_y + rotated_compass.shape[0]
        x1, x2 = compass_x, compass_x + rotated_compass.shape[1]

        alpha_s = rotated_compass[:, :, 3] / 255.0  # Alpha channel of the compass image
        alpha_l = 1.0 - alpha_s

        for c in range(0, 3):
            self.panorama_image[y1:y2, x1:x2, c] = (alpha_s * rotated_compass[:, :, c] +
                                                     alpha_l * self.panorama_image[y1:y2, x1:x2, c])

    def resize_compass(self):
        """Resize the compass image to fit in the corner of the panorama"""
        h, w, _ = self.panorama_image.shape
        compass_size = (int(w * 0.2), int(h * 0.2))  # Resize compass to 20% of the panorama dimensions
        return cv2.resize(self.compass_image, compass_size, interpolation=cv2.INTER_AREA)

    def rotate_image(self, image, angle):
        """Rotate the image by a specified angle"""
        center = (image.shape[1] // 2, image.shape[0] // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_image = cv2.warpAffine(image, rotation_matrix, (image.shape[1], image.shape[0]), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
        return rotated_image


def main(args=None):
    rclpy.init(args=args)
    node = PanoramaOverlayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
