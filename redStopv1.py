# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
rawCapture = PiRGBArray(camera)
# allow the camera to warmup
time.sleep(0.1)
while True:
    # grab an image from the camera
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    # display the image on screen and wait for a keypress
    cv2.imshow("Image", image)
    key=cv2.waitKey(1)&0xFF
    if key == ord("q"): # klavyeden q tuşuna basılırsa döngüyü sonlandır.
        break