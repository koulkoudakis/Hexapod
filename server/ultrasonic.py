#! /usr/bin/python
# File name   : ultrasonic.py
# Description : Detection distance and tracking with ultrasonic
# E-mail      : koulkoudakis@gmail.com
# Author      : Sharome Burton
# Date        : 2020/10/04

import RPi.GPIO as GPIO
import time

Tr = 11 # GPIO 11
Ec = 8  # GPIO 08

def ultdist():                # Reading distance using ultrasonic sensor
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(Tr, GPIO.OUT,initial=GPIO.LOW)
	GPIO.setup(Ec, GPIO.IN)

	GPIO.output(Tr, GPIO.HIGH)
	time.sleep(0.00001)
	GPIO.output(Tr, GPIO.LOW)

	while not GPIO.input(Ec):
		pass

	t1 = time.time()

	while GPIO.input(Ec):
		pass

	t2 = time.time()

	return (t2-t1)*343.0/2       # 1/2 distance travelled = speed of sound(343 m/s) * change in time (t2-t1) / 2

#try:
#       pass

#except KeyboardInterrupt:
#	GPIO.cleanup()