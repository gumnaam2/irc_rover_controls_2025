import cv2
import numpy as np
import imutils

def add_text_to_image(image, latitude, longitude, direction):
    """Add latitude, longitude, and direction information to the image."""
    # Add the latitude and longitude as text
    cv2.putText(image, f"Lat: {latitude:.6f}, Lon: {longitude:.6f}",
                (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Calculate the height-to-width ratio and add it as text
    h, w, _ = image.shape
    height_width_ratio = round(w/h)
    cv2.putText(image, f"Width/Height Ratio: {height_width_ratio:.2f}",
                (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255,0), 2)

    # Overlay the compass image if direction is provided
    if direction is not None:
        overlay_compass(image, direction)

    return image

def overlay_compass(image, direction):
    """Overlay a rotated compass image on the bottom-right corner of the panorama."""
    compass_image = cv2.imread('compass.png', cv2.IMREAD_UNCHANGED)
    
    if compass_image is None:
        print("Compass image not found!")
        return

    # Resize the compass to fit in the corner
    

    # Calculate the position to place the compass in the corner


    # Rotate the compass image to the given direction
    rotated_compass = rotate_image(compass_image, direction)
    compass_resized = resize_compass(image, rotated_compass)
    h, w, _ = image.shape
    compass_x = w - compass_resized.shape[1] - 30
    compass_y = h - compass_resized.shape[0] - 30
    # Overlay the compass onto the image using the alpha channel
    y1, y2 = compass_y, compass_y + compass_resized.shape[0]
    x1, x2 = compass_x, compass_x + compass_resized.shape[1]
    print(image.shape)

    # alpha_s = rotated_compass[:, :, 2] / 255.0  # Alpha channel
    # alpha_l = 1.0 - alpha_s

    # for c in range(0, 3):
    #     image[y1:y2, x1:x2, c] = (alpha_s * rotated_compass[:, :, c] +
    #                                alpha_l * image[y1:y2, x1:x2, c])
    image[y1:y2,x1:x2]=compass_resized

def resize_compass(image, compass_image):
    """Resize the compass image to fit in the bottom-right corner."""
    h, w, _ = image.shape
    compass_size = (int(h * 0.3), int(h * 0.3))  # Resize compass to 20% of panorama dimensions
    return cv2.resize(compass_image, compass_size, interpolation=cv2.INTER_AREA)

def rotate_image(image, angle):
    """Rotate the image by the specified angle."""
    center = (image.shape[1] // 2, image.shape[0] // 2)
    #rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    #rotated_image = cv2.warpAffine(image, rotation_matrix, (image.shape[1], image.shape[0]),
    #                               flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
    rotated_image=imutils.rotate_bound(image,angle)
    return rotated_image

def main():
    # Sample input values
    latitude = 37.7749   # Example latitude (San Francisco)
    longitude = -122.4194  # Example longitude (San Francisco)
    direction = 45.0  # Example direction (45 degrees - NE)

    # Load the panorama image from file (replace with your file path)
    panorama_image = cv2.imread('panorama.jpg')

    if panorama_image is None:
        print("Panorama image not found!")
        return

    # Add text (latitude, longitude, height/width ratio) and overlay compass
    modified_image = add_text_to_image(panorama_image, latitude, longitude, direction)

    # Save the modified image
    cv2.imwrite('output_panorama_with_overlay.jpg', modified_image)
    print("Modified image saved as 'output_panorama_with_overlay.jpg'.")

if __name__ == '__main__':
    main()
