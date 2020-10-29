#! /usr/bin/python
# File name   : laser.py
# Description : Laser beam
# E-mail      : koulkoudakis@gmail.com
# Author      : Sharome Burton
# Date        : 2020/10/12

import RPi.GPIO as GPIO
import time

La = 5 # GPIO 05

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(La, GPIO.OUT)

def laser(state):                # Activate laser
  
    if state == 0:
        GPIO.output(La, GPIO.LOW)
        #print('laser off')
    
    else:
        GPIO.output(La, GPIO.HIGH)
        #print('laser on')
        
def blink(OnTime, OffTime):      # Laser blinks at custom interval
    while True:
            laser(1)
            time.sleep(OnTime)
            laser(0)
            time.sleep(OffTime)
        
def destroy():
    GPIO.output(La, GPIO.LOW)
    GPIO.cleanup()
    
if __name__ == "__main__":
    while True:
        setup()
        
        try:
            blink(1,0.5)
            
        except KeyboardInterrupt:  
            destroy()
