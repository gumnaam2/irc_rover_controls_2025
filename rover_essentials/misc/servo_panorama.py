import lgpio
from time import sleep
import threading 
import traceback

import cv2
from stitching import Stitcher
# pip install stitching 

#Setting the pin to give PWM on 
pin=18
freq=50
#Setting PWM according to the Servo motor chosen 
h = lgpio.gpiochip_open(0)
def Turn():
    global panorama_ready
    try:
        i=0
        while True:
            angle_to_point=(15*i-90)
            i+=1
            if i==13:
                i=0
            duty = ((angle_to_point/90)*5)+7.5
            if duty<2.5:
                duty=2.5
            #print(duty)
            lgpio.tx_pwm(h, pin, freq, duty)
            sleep(0.2)
            if duty==2.5:
                sleep(0.8)
            if panorama_ready:
                print("Images captured...Stopping Servo")
                break
    except (KeyboardInterrupt, SystemExit):
        print('Exit due to keyboard interrupt')
    except Exception as ex:
        print('Python error with no Exception handler:')
        print('Traceback error:', ex)
        traceback.print_exc()
    finally:
        return

imgs = [] 
panorama_ready=False
def capture(cam):
    global imgs
    ret, frame = cam.read()
    if not ret:
        print("failed to grab frame")
        return
    frame=cv2.resize(frame,(0,0),fx=0.9,fy=0.9) 
    ''''cv2.imshow("frame",frame)
    cv2.waitKey(0)'''
    imgs.append(frame)
    return

def setup_camera(frame,wait_time):
    global imgs,panorama_ready
    cam = cv2.VideoCapture(0)
   # cv2.namedWindow("Webcam")
    print("Capturing Panorama")
    print(f'The cam is {cam.isOpened()}')
    for i in range(frame):
        capture(cam)
        #print("move now")
        sleep(wait_time)

    print(len(imgs))
    panorama_ready=True
    stitcher = Stitcher(detector="sift", confidence_threshold=0.9)

    """    
        stitchy=cv2.Stitcher.create() 
    (dummy,panorama)=stitchy.stitch(imgs) 
    
    if dummy != cv2.STITCHER_OK: 
        print("stitching ain't successful") 
    else:  
        print('Your Panorama is ready!!!') """

    # final output 
    print("Now processing panorama")
    panorama = stitcher.stitch(imgs)
   # cv2.imshow('final result',panorama)
    cv2.imwrite('panorama.jpg',panorama) 
    cv2.waitKey(0)

    cam.release()
    cam.destroyAllWindows()

class Panorama(threading.Thread):
    def __init__(self, frames,wait_time,is_servo):
        threading.Thread.__init__(self)
        self.num_frames=frames
        self.wait_time=wait_time
        self.is_servo=is_servo
    def run(self):
        if self.is_servo:
            print("Starting servo")
            Turn()
        elif not self.is_servo:
            setup_camera(self.num_frames,self.wait_time)

if __name__ == '__main__':
    servo_thread = Panorama(50,0.1,True)
    camera_thread = Panorama(50,0.1,False)

    servo_thread.start()
    camera_thread.start()
    print()
    print("Active threads", threading.activeCount())
