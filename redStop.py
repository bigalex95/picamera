# from picamera.array import PiRGBArray
# from picamera import PiCamera
# import RPi.GPIO as GPIO
import time
import cv2
import numpy as np

# GPIO.setmode(GPIO.BCM)

leftForward = 26
leftBack = 19
leftSpeed = 18
rightForward = 13
rightBack = 6
rightSpeed = 12
temp1 = 1

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(leftForward, GPIO.OUT)
# GPIO.setup(leftBack, GPIO.OUT)
# GPIO.setup(leftSpeed, GPIO.OUT)
# GPIO.setup(rightForward, GPIO.OUT)
# GPIO.setup(rightBack, GPIO.OUT)
# GPIO.setup(rightSpeed, GPIO.OUT)
# pl = GPIO.PWM(leftSpeed, 1000)
# pr = GPIO.PWM(rightSpeed, 1000)
# pl.start(25)
# pr.start(25)


 
def main():
    #kamerayı başlat 
    camera=PiCamera()
    camera.resolution=(640,480)
    camera.framerate=32
    rawCapture=PiRGBArray(camera,size=(640,480))
 
    #kamera için bekleme
    time.sleep(0.1)
    
    

    
 
    for frame in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):
        #resimi temsil eden numPy dizisini alıp imgOriginal içine atıyoruz
 
        imgOriginal=frame.array
        lower_hsv=np.array([155,50,50])
        upper_hsv=np.array([182,250,250])
     
        converted=cv2.cvtColor(imgOriginal,cv2.COLOR_BGR2HSV)
     
        mask_hsv=cv2.inRange(converted,lower_hsv,upper_hsv)
        output_hsv=cv2.bitwise_and(imgOriginal,imgOriginal,mask=mask_hsv)
        
        cv2.imshow("imgOriginal",imgOriginal)
        cv2.imshow("HSV",output_hsv)
 
        key=cv2.waitKey(1)&0xFF
 
        rawCapture.truncate(0) # birsonraki frame için rawCapture temizliyoruz.
 
        
        if key == ord("q"): # klavyeden q tuşuna basılırsa döngüyü sonlandır.
            return
        
while(1):
    x = 2
    
    if x == 1:
        print("ileri")
        pl.ChangeDutyCycle(60)
        pr.ChangeDutyCycle(60)
        GPIO.output(leftForward, GPIO.HIGH)
        GPIO.output(leftBack, GPIO.LOW)
        GPIO.output(rightForward, GPIO.LOW)
        GPIO.output(rightBack, GPIO.HIGH)
        temp1 = 1
        break



    elif x == 2:
        print("dur")
        pl.ChangeDutyCycle(60)
        pr.ChangeDutyCycle(60)
        GPIO.output(leftForward, GPIO.LOW)
        GPIO.output(leftBack, GPIO.LOW)
        GPIO.output(rightForward, GPIO.LOW)
        GPIO.output(rightBack, GPIO.LOW)
        break



    elif x == 3:
        print("geri")
        pl.ChangeDutyCycle(60)
        pr.ChangeDutyCycle(60)
        GPIO.output(leftForward, GPIO.LOW)
        GPIO.output(leftBack, GPIO.HIGH)
        GPIO.output(rightForward, GPIO.HIGH)
        GPIO.output(rightBack, GPIO.LOW)
        temp1 = 0
        break

       


    elif x == 4:
        print("sag")
        pl.ChangeDutyCycle(0)
        pr.ChangeDutyCycle(60)
        GPIO.output(leftForward, GPIO.HIGH)
        GPIO.output(leftBack, GPIO.LOW)
        GPIO.output(rightForward, GPIO.LOW)
        GPIO.output(rightBack, GPIO.HIGH)
        break


    elif x == '5':
        print("sol")
        pl.ChangeDutyCycle(60)
        pr.ChangeDutyCycle(0)
        GPIO.output(leftForward, GPIO.HIGH)
        GPIO.output(leftBack, GPIO.LOW)
        GPIO.output(rightForward, GPIO.LOW)
        GPIO.output(rightBack, GPIO.HIGH)
        break
        

 
if __name__ == "__main__":
        main()

