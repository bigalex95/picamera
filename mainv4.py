#serit takip
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
from picamera import PiCamera
import time
import cv2
import numpy as np
import math
#serihaberlesme
import serial
#trafik lambası
import imutils
#serit takip
GPIO.setmode(GPIO.BCM)
leftForward = 26
leftBack = 19
leftSpeed = 18
rightForward = 13
rightBack = 6
rightSpeed = 12
temp1 = 1
GPIO.setup(leftForward, GPIO.OUT)
GPIO.setup(leftBack, GPIO.OUT)
GPIO.setup(leftSpeed, GPIO.OUT)
GPIO.setup(rightForward, GPIO.OUT)
GPIO.setup(rightBack, GPIO.OUT)
GPIO.setup(rightSpeed, GPIO.OUT)
pl = GPIO.PWM(leftSpeed, 1000)
pr = GPIO.PWM(rightSpeed, 1000)
pl.start(25)
pr.start(25)
#serihaberlesme
referans=22
if __name__ == '__main__':
    #serit takip
    theta=0
    minLineLength = 5
    maxLineGap = 10
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 30
    rawCapture = PiRGBArray(camera, size=(640, 480))
    time.sleep(0.1)
    threshold=6
    #serihaberlesme
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()
    #trafik lambası
    trafiklambasi=0
    step=0


    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        #resimi temsil eden numPy dizisini alıp imgOriginal içine atıyoruz
        image = frame.array
        lower_hsv=np.array([165,50,50])
        upper_hsv=np.array([180,250,250])
     
        converted=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
     
        mask_hsv=cv2.inRange(converted,lower_hsv,upper_hsv)
        output_hsv=cv2.bitwise_and(image,image,mask=mask_hsv)
        
        img = cv2.medianBlur(output_hsv, 5)
        gray = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)        
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
      

        circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,
                                    param1=50,param2=30,minRadius=0,maxRadius=0)
        print(circles)
        step+=1
        if step==20:
            step=0
            trafiklambasi=0
        if step==0 and circles is not None:
            trafiklambasi=1

        image = frame.array
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 85, 85)
        mask = np.zeros_like(edged)
        print(image.shape)
        vertices = np.array([[(0,480),(320,250),(680,480)]],np.int32)
        cv2.fillPoly(mask, vertices, 255)
        masked = cv2.bitwise_and(edged, mask)
        lines = cv2.HoughLinesP(masked,2,np.pi/180,50,minLineLength,maxLineGap)
        if lines is not None:
            for x in range(0, len(lines)):
                for x1,y1,x2,y2 in lines[x]:
                    cv2.line(image,(x1,y1),(x2,y2),(250,0,0),3)
                    img = cv2.addWeighted(image,0.8,image, 1.0,0.)
                    theta=theta+math.atan2((y2-y1),(x2-x1))
        if ser.in_waiting > 0:
            mesafe = ser.readline().decode('utf-8').rstrip()
            #print("Mesafe")
            print(type(mesafe))
            if(trafiklambasi):
                pl.ChangeDutyCycle(0)
                pr.ChangeDutyCycle(0)
                GPIO.output(leftForward, GPIO.HIGH)
                GPIO.output(leftBack, GPIO.LOW)
                GPIO.output(rightForward, GPIO.HIGH)
                GPIO.output(rightBack, GPIO.LOW)
                print('Trafik kırmızı')
            elif((int(mesafe)<30 and int(mesafe)>0)):
                pl.ChangeDutyCycle(0)
                pr.ChangeDutyCycle(0)
                GPIO.output(leftForward, GPIO.HIGH)
                GPIO.output(leftBack, GPIO.LOW)
                GPIO.output(rightForward, GPIO.HIGH)
                GPIO.output(rightBack, GPIO.LOW)
                print('engel var dur')
            else:
                if(theta>threshold):
                    pl.ChangeDutyCycle(0)
                    pr.ChangeDutyCycle(55)
                    GPIO.output(leftForward, GPIO.HIGH)
                    GPIO.output(leftBack, GPIO.LOW)
                    GPIO.output(rightForward, GPIO.HIGH)
                    GPIO.output(rightBack, GPIO.LOW)
                    print("sol")
                elif(theta<-threshold):
                    pl.ChangeDutyCycle(55)
                    pr.ChangeDutyCycle(0)
                    GPIO.output(leftForward, GPIO.HIGH)
                    GPIO.output(leftBack, GPIO.LOW)
                    GPIO.output(rightForward, GPIO.HIGH)
                    GPIO.output(rightBack, GPIO.LOW)
                    print("sag")  
                elif(abs(theta)<threshold):
                    pl.ChangeDutyCycle(50)
                    pr.ChangeDutyCycle(50)
                    GPIO.output(leftForward, GPIO.HIGH)
                    GPIO.output(leftBack, GPIO.LOW)
                    GPIO.output(rightForward, GPIO.HIGH)
                    GPIO.output(rightBack, GPIO.LOW)
                    print("düz") 
    
        theta=0
        cv2.imshow("Frame",image)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        if key == ord("q"):
            break