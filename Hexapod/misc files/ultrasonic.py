#! /usr/bin/python
# File name   : ultrasonic.py
# Description : Detection distance and tracking with ultrasonic
# E-mail      : koulkoudakis@gmail.com
# Author      : Sharome Burton
# Date        : 2020/10/04

import RPi.GPIO as GPIO
import time



def setup():        # Sets up GPIO pins
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)   # Triggers ultrasonic buzzer
    GPIO.setup(Ec, GPIO.IN)                     # Waits for echo

def ultdist():      # Reading distance using ultrasonic sensor
    GPIO.output(Tr, GPIO.HIGH)
    #print('trigger')
    time.sleep(0.00001)
    GPIO.output(Tr, GPIO.LOW)

    while not GPIO.input(Ec):
        pass
	#print('not yet')


    t1 = time.time()
    #print("Receive started")

    while GPIO.input(Ec):
	#print('echo')
        pass

    t2 = time.time()


    return (t2-t1)*34300.0/2       # 1/2 distance travelled(cm) = speed of sound(343 m/s) * change in time (t2-t1) / 2

if __name__ == "__main__":
    while True:
        print(f"distance: {ultdist():.2f}cm")
        time.sleep(0.5)
#try:
#       pass

#except KeyboardInterrupt:
#	GPIO.cleanup()
