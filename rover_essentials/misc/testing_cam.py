import cv2

cam_port = 0
cam = cv2.VideoCapture(cam_port)

result, image = cam.read()

cv2.imwrite("imasdf_now.png", image)
