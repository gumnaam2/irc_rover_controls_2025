"""pub_sub_broadcast.py -- broadcast OpenCV stream using PUB SUB."""
import sys
import socket
import traceback
from time import sleep
import cv2
from imutils.video import VideoStream
import imagezmq
import threading 
class camThread(threading.Thread):
    def __init__(self, previewName, camID,port,jpeg_quality):
        threading.Thread.__init__(self)
        self.previewName = previewName
        self.camID = camID
        self.port=port
        self.jpeg=jpeg_quality
    def run(self):
        print("Starting " + self.previewName)
        camPreview(self.previewName, self.camID,self.port,self.jpeg)


def camPreview(previewName, camID,port,jpeg_quality):
        # Publish on port
        #port=input("Enter port")
        sender = imagezmq.ImageSender("tcp://*:{}".format(port), REQ_REP=False)

        # Open input stream; comment out one of these capture = VideoStream() lines!
        # *** You must use only one of Webcam OR PiCamera
        # Webcam source for broadcast images
        #src=int(input("enter video source"))
        capture = VideoStream(src=camID)  # Webcam
        # PiCamera source for broadcast images (Raspberry Pi only)
        # capture = VideoStream(usePiCamera=True)  # PiCamera

        capture.start()
        sleep(2.0)  # Warmup time; needed by PiCamera on some RPi's
        print("Input stream opened")

        # JPEG quality, 0 - 100
        # Send RPi hostname with each image
        # This might be unnecessary in this pub sub mode, as the receiver will
        #    already need to know our address and can therefore distinguish streams
        # Keeping it anyway in case you wanna send a meaningful tag or something
        #    (or have a many to many setup)
        rpi_name = socket.gethostname()

        try:
            counter = 0
            while True:
                frame = capture.read()
                #if camID==2:
                #        print(frame)
                if frame is not None:
                    #print(f"I am sending {camID}")
                    ret_code, jpg_buffer = cv2.imencode(
                        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
                    sender.send_jpg(rpi_name, jpg_buffer)
                    #sleep(0.2)
                #print("Sent frame {}".format(counter))
                #counter = counter + 1
        except (KeyboardInterrupt, SystemExit):
            print('Exit due to keyboard interrupt')
        except Exception as ex:
            print('Python error with no Exception handler:')
            print('Traceback error:', ex)
            traceback.print_exc()
        finally:
            capture.stop()
            sender.close()
            sys.exit()

if __name__ == "__main__":
    thread1 = camThread("Camera 1", 0,5555,80)
    #thread3 = camThread("Camera 2", 2,5566,10)

    thread1.start()
   # thread3.start()
    print()
    print("Active threads", threading.activeCount())
