import serial
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
leftForward = 26
leftBack = 19
leftSpeed = 18
rightForward = 13
rightBack = 6
rightSpeed = 12
referans=22

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



if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()
    while True:
        if ser.in_waiting > 0:
            mesafe = ser.readline().decode('utf-8').rstrip()
                    #print("Mesafe")
            print(type(mesafe))
                
            if(int(mesafe)<30):
                pl.ChangeDutyCycle(0)
                pr.ChangeDutyCycle(0)
                GPIO.output(leftForward, GPIO.HIGH)
                GPIO.output(leftBack, GPIO.LOW)
                GPIO.output(rightForward, GPIO.HIGH)
                GPIO.output(rightBack, GPIO.LOW)
                print('engel var dur')
            if(int(mesafe)>30):
                pl.ChangeDutyCycle(55)
                pr.ChangeDutyCycle(55)
                GPIO.output(leftForward, GPIO.HIGH)
                GPIO.output(leftBack, GPIO.LOW)
                GPIO.output(rightForward, GPIO.HIGH)
                GPIO.output(rightBack, GPIO.LOW)
                print('ileri')
         
      
            
