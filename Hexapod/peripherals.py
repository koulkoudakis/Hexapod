'''
Author: Sharome Burton
Filename: peripherals.py
Description: Governs accelerometer/gyroscope, ultrasonic distance sensor, 
laser emitter and other small devices
'''

import RPi.GPIO as GPIO
import serial
from mpu6050 import mpu6050
import time

Tr = 11 # GPIO 11 - Trigger output ultrasonic sensor
Ec = 8  # GPIO 08 - Echo input ultrasonic senor
La = 5  # GPIO 05 - Activate Laser

class Peripheral:
    def __init__(self, name, status=1):
        self.name = name
        self.status = status
        if self.status:
            print(f'{self.name} initialized and activated')
        else:
            print(f'{self.name} initialized but inactive')
            
    def switch(self):
        if self.status:
            self.status = 0
            print(f'{self.name} deactivated')
        else:
            self.status = 1
            print(f'{self.name} activated')


'''
External inertial measurement unit connected via USB
'''            
class exIMU(Peripheral):
    def __init__(self, name, status, port="/dev/ttyACM0",baud=115200,
                timeout=0.1):
        super().__init__(name, status)
        self.port = port
        self.baud = baud
        self.timeout = timeout
        
        
        self.data = ""
        self.heading = 0.0
        self.accX = 0.0
        self.accY = 0.0
        self.accZ = 0.0
        
        if self.status:
            self.setup(1)
        else:
            self.setup(0)
            
    def setup(self, status):
        if status:
            try:
                self.sensor = serial.Serial(self.port,
                            baudrate=self.baud, timeout=self.timeout)
                self.BNO055 = 1
            except:
                self.BNO055 = 0
                print('BNO055 IMU failed to initialize')
        else:
            if self.BNO055:
                self.sensor.__del__() # close port when instance is freed
            else:
                pass
            
            
    def read(self):
        if (self.sensor.in_waiting > 0): # Checks number of bytes in 
                                         # input buffer
                                         
            self.data = str(self.sensor.readline(), "utf-8").split(',')
            if len(self.data) == 16:
                self.heading = float(self.data[-1]) # Compass heading
                if self.heading < 0:                # convert negative angles
                    self.heading += 360.0
                    
                self.accX = float(self.data[0]) # Acceleration in X,
                self.accY = float(self.data[1]) # Y,
                self.accZ = float(self.data[2]) # Z direction
                
                #print(f'Heading:{self.heading}, X:{self.accX} Y:{self.accY} Z:{self.accZ}')
                self.sensor.reset_input_buffer() 
                return [self.heading, self.accX, self.accY, self.accZ]
                       
    

'''
Internal Accelerometer/Gyroscope
'''

class AccGyro(Peripheral):
    def __init__(self, name, status):
        super().__init__(name, status)
        if self.status:
            self.setup(1)
        else:
            self.setup(0)
            
            self.accel = {}
            self.gyro = {}
            self.temp = 0.0
            
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0

    def setup(self, status):
        if status:
            try:
                self.sensor = mpu6050(0x68)
                self.mpu6050_connection = 1
            except:
                self.mpu6050_connection = 0
                print('MPU6050 failed to initialize')
        else:
            pass

    def mpu6050_test(self):
        self.accelerometer_data = self.sensor.get_accel_data()
        print('X=%f,Y=%f,Z=%f' % (self.accelerometer_data['x'], self.accelerometer_data['y'],
                                  self.accelerometer_data['z']))
        time.sleep(0.3)
        
    def read(self):
        self.accel, self.gyro, self.temp = self.sensor.get_all_data()

'''
Distance estimation using ultrasonic sensor
'''

class Ultrasonic(Peripheral):
    def __init__(self, name, status):
        super().__init__(name, status)
        if self.status:
            self.setup(1)
        else:
            self.setup(0)

    def setup(self, status):  # Sets up GPIO pins
        if status:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(Tr, GPIO.OUT, initial=GPIO.LOW)  # Triggers ultrasonic buzzer
            GPIO.setup(Ec, GPIO.IN)  # Waits for echo
        else:
            GPIO.cleanup()  # Resets GPIO pins used in program to default state

    def ultdist(self):  # Reading distance using ultrasonic sensor
        GPIO.output(Tr, GPIO.HIGH)
        # print('trigger')
        time.sleep(0.00001)
        GPIO.output(Tr, GPIO.LOW)

        while not GPIO.input(Ec):
            pass
        # print('not yet')

        t1 = time.time()
        # print("Receive started")

        while GPIO.input(Ec):
            # print('echo')
            pass

        t2 = time.time()

        return (t2 - t1) * 34300.0 / 2  # 1/2 distance travelled(cm) = speed of sound(343 m/s)
                                        # * change in time (t2-t1) / 2

'''
Laser emitter
'''
class Laser(Peripheral):
    def __init__(self, name, status):
        super().__init__(name, status)
        if self.status:
            self.setup(1)
        else:
            self.setup(0)


    def setup(self, status):
        if status:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(La, GPIO.OUT)
        else:
            self.destroy()

    def laser(self, state):  # Activate laser

        if state == 0:
            GPIO.output(La, GPIO.LOW)
            # print('laser off')

        else:
            GPIO.output(La, GPIO.HIGH)
            # print('laser on')

    def blink(self, OnTime, OffTime):  # Laser blinks at custom interval
        while True:
            self.laser(1)
            time.sleep(OnTime)
            self.laser(0)
            time.sleep(OffTime)

    def destroy(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(La, GPIO.OUT)
        GPIO.output(La, GPIO.LOW)   # Turn off laser
        GPIO.cleanup()  # Resets GPIO pins used in program to default state


    # Instantiation of peripheral devices
    
    #ultra = Ultrasonic('Ultrasonic Sensor', 1)
    #laser = Laser('Laser Emitter', 1)
    #IMU = AccGyro('IMU', 1)


if __name__ == "__main__":
    
    
    # Instantiation of peripheral devices
    
    ultra = Ultrasonic('Ultrasonic Sensor', 1)
    laser = Laser('Laser Emitter', 1)
    IMU = AccGyro('IMU', 1)
    compass = exIMU('Compass',1)

    while True:
        '''
        
        print(f"distance: {ultra.ultdist():.2f}cm")
        time.sleep(0.5)
        
        '''

        '''
        
        try:
            laser.blink(1,0.5)
            
        except KeyboardInterrupt:  
            laser.destroy()
            
        '''

        '''
        
        IMU.mpu6050_test()
        
        '''
        
        
        compass.read()

        pass
