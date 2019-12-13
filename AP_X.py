import cv2
import numpy as np
import math
import time
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
from picamera import PiCamera

def nothing(x):
    pass

camera = PiCamera()
rawCapture = PiRGBArray(camera)
camera.rotation = 180

in1 = 23
in2 = 24
ena = 18
in3 = 22
in4 = 27
enb = 13

p1=GPIO.PWM(ena,1000)
p2=GPIO.PWM(enb,1000)
p1.start(5)
p2.start(5)

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(ena,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enb,GPIO.OUT)

def forward():
    p1.ChangeDutyCycle(12)
    p2.ChangeDutyCycle(12)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

def left():
    p1.ChangeDutyCycle(47)
    p2.ChangeDutyCycle(25)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)

def right():
    p1.ChangeDutyCycle(25)
    p2.ChangeDutyCycle(47)
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)

theta=0
minLineLength = 10
maxLineGap = 10

lower_red = np.array([150,70,50])
upper_red = np.array([180,255,255])

try:
    Size_crop = [0,0]
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image2 = frame.array
        image = cv2.GaussianBlur(image2, (5, 5), 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_red, upper_red)
        res = cv2.bitwise_and(image,image, mask=mask)
        edged = cv2.Canny(res, 85, 85)
        lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
        rawCapture.truncate(0)
        if(lines is not None):
            for line in lines:
                for x1,y1,x2,y2 in line:
                    theta=theta+math.atan2((y2-y1),(x2-x1))
            theta /= len(lines)

        font = cv2.FONT_HERSHEY_COMPLEX

        if(-3.142<theta<-2.094 or 0<theta<1.047):
            left()
            print("left", theta)
            leftt = "left:" + str(theta)

        elif(2.094<theta<3.142 or -1.047<theta<0):
            right()
            print("right", theta)
            rightt = "right:" + str(theta)
        
        else:
            forward()
            print("straight", theta)
            straightt = "straight:" + str(theta)
        
        theta = 0
        cv2.imshow("Res",res)
         k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()
    cap.release()

except KeyboardInterrupt:
    GPIO.cleanup() 