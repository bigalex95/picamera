from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
from shapedetector import ShapeDetector


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

        # load the image and resize it to a smaller factor so that
        # the shapes can be approximated better
        resized = imutils.resize(output_hsv, width=300)
        ratio = output_hsv.shape[0] / float(resized.shape[0])
        # convert the resized image to grayscale, blur it slightly,
        # and threshold it
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        # find contours in the thresholded image and initialize the
        # shape detector
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        sd = ShapeDetector()

        # loop over the contours
        for c in cnts:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = sd.detect(c)
            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(output_hsv, [c], -1, (0, 255, 0), 2)
            cv2.putText(output_hsv, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 2)
            # show the output image
            cv2.imshow("Image", output_hsv)
            cv2.waitKey(0)
 
        cv2.imshow("imgOriginal",imgOriginal)
        cv2.imshow("HSV",output_hsv)

        rawCapture.truncate(0) # birsonraki frame için rawCapture temizliyoruz.
        
        key=cv2.waitKey(1)&0xFF
        if key == ord("q"): # klavyeden q tuşuna basılırsa döngüyü sonlandır.
            return
 

if __name__ == "__main__":
        main()