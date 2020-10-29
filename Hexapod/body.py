'''
Author: Sharome Burton
Filename: body.py
Description: Governs servo movement, locomotion
'''

import time
import Adafruit_PCA9685
from mpu6050 import mpu6050
import threading
import RPIservo



class Body:

    def __init__(self, name, reverse=0, *args, **kwargs):
        # Name
        self.name = name        
        # Settings
        if reverse:
            # Direction of legs
            self.leftSide_direction = 0
            self.rightSide_direction = 1

            # Height of legs
            self.leftSide_height = 1
            self.rightSide_height = 0

            # Direction of head servos
            self.Up_Down_direction = 0
            self.Left_Right_direction = 0

        else:
            self.leftSide_direction = 1
            self.rightSide_direction = 0

            self.leftSide_height = 0
            self.rightSide_height = 1

            self.Up_Down_direction = 1
            self.Left_Right_direction = 1

        # Set the range of height for legs

        self.height_change = 30

        # Maximum ranges for head servos
        self.Left_Right_Max = 500
        self.Left_Right_Min = 100
        self.Up_Down_Max = 500
        self.Up_Down_Min = 270
        self.Left_Right_input = 300
        self.Up_Down_input = 300

        self.look_wiggle = 30  # Head movement increment using 'look' command
        self.move_stu = 1  # Enables smooth servo movement if smoothMode = 1

        self.DPI = 17  # DPI for smooth servo movement (set higher for smoother movement)

        # Movement status
        self.step_set = 1
        self.direction_command = 'no'
        self.turn_command = 'no'

        # Modes
        self.smoothMode = 0
        self.balanceMode = 0

        # Set-up Pulse Width Modulation for servos (PWM)
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)

        # Set a default pwm value for all servos.

        for i in range(0, 16):
            exec('self.pwm%d=RPIservo.init_pwm%d' % (i, i))

        # Accelerometer/Gyroscope
        '''
        try:
            self.sensor = mpu6050(0x68)
            self.mpu6050_connection = 1
        except:
            self.mpu6050_connection = 0
        '''

    '''
    Initialize all servos
    '''

    def init_all(self):
        self.pwm.set_pwm(0, 0, self.pwm0)
        self.pwm.set_pwm(1, 0, self.pwm1)
        self.pwm.set_pwm(2, 0, self.pwm2)
        self.pwm.set_pwm(3, 0, self.pwm3)

        self.pwm.set_pwm(4, 0, self.pwm4)
        self.pwm.set_pwm(5, 0, self.pwm5)
        self.pwm.set_pwm(6, 0, self.pwm6)
        self.pwm.set_pwm(7, 0, self.pwm7)

        self.pwm.set_pwm(8, 0, self.pwm8)
        self.pwm.set_pwm(9, 0, self.pwm9)
        self.pwm.set_pwm(10, 0, self.pwm10)
        self.pwm.set_pwm(11, 0, self.pwm11)

        self.pwm.set_pwm(12, 0, self.pwm12)
        self.pwm.set_pwm(13, 0, self.pwm13)
        self.pwm.set_pwm(14, 0, self.pwm14)
        self.pwm.set_pwm(15, 0, self.pwm15)

    '''
    Prevents servos from exceeding limits on range of motion
    '''
    def ctrl_range(self, raw, max_genout, min_genout):
        if raw > max_genout:
            raw_output = max_genout
        elif raw < min_genout:
            raw_output = min_genout
        else:
            raw_output = raw
        return int(raw_output)

    '''
    left_I   -<forward>-- right_III
    left_II  ---<BODY>---  right_II
    left_III -<Backward>-   right_I

                pos=1
               /     \
              /       \
             /         \
        pos=2---pos=3---pos=4

    Change the value of wiggle to set the range and direction that the legs moves.
    '''

    def left_I(self, pos, wiggle, heightAdjust=0):
        if pos == 0:
            # self.pwm.set_pwm(0,0,self.pwm0)
            if self.leftSide_height:
                self.pwm.set_pwm(1, 0, self.pwm1 + heightAdjust)
            else:
                self.pwm.set_pwm(1, 0, self.pwm1 - heightAdjust)
        else:
            if self.leftSide_direction:
                if pos == 1:
                    self.pwm.set_pwm(0, 0, self.pwm0)
                    if self.leftSide_height:
                        self.pwm.set_pwm(1, 0, self.pwm1 + 3 * self.height_change)
                    else:
                        self.pwm.set_pwm(1, 0, self.pwm1 - 3 * self.height_change)
                elif pos == 2:
                    self.pwm.set_pwm(0, 0, self.pwm0 + wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(1, 0, self.pwm1 - self.height_change)
                    else:
                        self.pwm.set_pwm(1, 0, self.pwm1 + self.height_change)
                elif pos == 3:
                    self.pwm.set_pwm(0, 0, self.pwm0)
                    if self.leftSide_height:
                        self.pwm.set_pwm(1, 0, self.pwm1 - self.height_change)
                    else:
                        self.pwm.set_pwm(1, 0, self.pwm1 + self.height_change)
                elif pos == 4:
                    self.pwm.set_pwm(0, 0, self.pwm0 - wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(1, 0, self.pwm1 - self.height_change)
                    else:
                        self.pwm.set_pwm(1, 0, self.pwm1 + self.height_change)
            else:
                if pos == 1:
                    self.pwm.set_pwm(0, 0, self.pwm0)
                    if self.leftSide_height:
                        self.pwm.set_pwm(1, 0, self.pwm1 + 3 * wiggle)
                    else:
                        self.pwm.set_pwm(1, 0, self.pwm1 - 3 * wiggle)
                elif pos == 2:
                    self.pwm.set_pwm(0, 0, self.pwm0 - wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(1, 0, self.pwm1 - wiggle)
                    else:
                        self.pwm.set_pwm(1, 0, self.pwm1 + wiggle)
                elif pos == 3:
                    self.pwm.set_pwm(0, 0, self.pwm0)
                    if self.leftSide_height:
                        self.pwm.set_pwm(1, 0, self.pwm1 - wiggle)
                    else:
                        self.pwm.set_pwm(1, 0, self.pwm1 + wiggle)
                elif pos == 4:
                    self.pwm.set_pwm(0, 0, self.pwm0 + wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(1, 0, self.pwm1 - wiggle)
                    else:
                        self.pwm.set_pwm(1, 0, self.pwm1 + wiggle)

    def left_II(self, pos, wiggle, heightAdjust=0):
        if pos == 0:
            # self.pwm.set_pwm(2,0,self.pwm2)
            if self.leftSide_height:
                self.pwm.set_pwm(3, 0, self.pwm3 + heightAdjust)
            else:
                self.pwm.set_pwm(3, 0, self.pwm3 - heightAdjust)
        else:
            if self.leftSide_direction:
                if pos == 1:
                    self.pwm.set_pwm(2, 0, self.pwm2)
                    if self.leftSide_height:
                        self.pwm.set_pwm(3, 0, self.pwm3 + 3 * self.height_change)
                    else:
                        self.pwm.set_pwm(3, 0, self.pwm3 - 3 * self.height_change)
                elif pos == 2:
                    self.pwm.set_pwm(2, 0, self.pwm2 + wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(3, 0, self.pwm3 - self.height_change)
                    else:
                        self.pwm.set_pwm(3, 0, self.pwm3 + self.height_change)
                elif pos == 3:
                    self.pwm.set_pwm(2, 0, self.pwm2)
                    if self.leftSide_height:
                        self.pwm.set_pwm(3, 0, self.pwm3 - self.height_change)
                    else:
                        self.pwm.set_pwm(3, 0, self.pwm3 + self.height_change)
                elif pos == 4:
                    self.pwm.set_pwm(2, 0, self.pwm2 - wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(3, 0, self.pwm3 - self.height_change)
                    else:
                        self.pwm.set_pwm(3, 0, self.pwm3 + self.height_change)
            else:
                if pos == 1:
                    self.pwm.set_pwm(2, 0, self.pwm2)
                    if self.leftSide_height:
                        self.pwm.set_pwm(3, 0, self.pwm3 + 3 * wiggle)
                    else:
                        self.pwm.set_pwm(3, 0, self.pwm3 - 3 * wiggle)
                elif pos == 2:
                    self.pwm.set_pwm(2, 0, self.pwm2 - wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(3, 0, self.pwm3 - wiggle)
                    else:
                        self.pwm.set_pwm(3, 0, self.pwm3 + wiggle)
                elif pos == 3:
                    self.pwm.set_pwm(2, 0, self.pwm2)
                    if self.leftSide_height:
                        self.pwm.set_pwm(3, 0, self.pwm3 - wiggle)
                    else:
                        self.pwm.set_pwm(3, 0, self.pwm3 + wiggle)
                elif pos == 4:
                    self.pwm.set_pwm(2, 0, self.pwm2 + wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(3, 0, self.pwm3 - wiggle)
                    else:
                        self.pwm.set_pwm(3, 0, self.pwm3 + wiggle)

    def left_III(self, pos, wiggle, heightAdjust=0):
        if pos == 0:
            # self.pwm.set_pwm(4,0,self.pwm4)
            if self.leftSide_height:
                self.pwm.set_pwm(5, 0, self.pwm5 + heightAdjust)
            else:
                self.pwm.set_pwm(5, 0, self.pwm5 - heightAdjust)
        else:
            if self.leftSide_direction:
                if pos == 1:
                    self.pwm.set_pwm(4, 0, self.pwm4)
                    if self.leftSide_height:
                        self.pwm.set_pwm(5, 0, self.pwm5 + 3 * self.height_change)
                    else:
                        self.pwm.set_pwm(5, 0, self.pwm5 - 3 * self.height_change)
                elif pos == 2:
                    self.pwm.set_pwm(4, 0, self.pwm4 + wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(5, 0, self.pwm5 - self.height_change)
                    else:
                        self.pwm.set_pwm(5, 0, self.pwm5 + self.height_change)
                elif pos == 3:
                    self.pwm.set_pwm(4, 0, self.pwm4)
                    if self.leftSide_height:
                        self.pwm.set_pwm(5, 0, self.pwm5 - self.height_change)
                    else:
                        self.pwm.set_pwm(5, 0, self.pwm5 + self.height_change)
                elif pos == 4:
                    self.pwm.set_pwm(4, 0, self.pwm4 - wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(5, 0, self.pwm5 - self.height_change)
                    else:
                        self.pwm.set_pwm(5, 0, self.pwm5 + self.height_change)
            else:
                if pos == 1:
                    self.pwm.set_pwm(4, 0, self.pwm4)
                    if self.leftSide_height:
                        self.pwm.set_pwm(5, 0, self.pwm5 + 3 * wiggle)
                    else:
                        self.pwm.set_pwm(5, 0, self.pwm5 - 3 * wiggle)
                elif pos == 2:
                    self.pwm.set_pwm(4, 0, self.pwm4 - wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(5, 0, self.pwm5 - wiggle)
                    else:
                        self.pwm.set_pwm(5, 0, self.pwm5 + wiggle)
                elif pos == 3:
                    self.pwm.set_pwm(4, 0, self.pwm4)
                    if self.leftSide_height:
                        self.pwm.set_pwm(5, 0, self.pwm5 - wiggle)
                    else:
                        self.pwm.set_pwm(5, 0, self.pwm5 + wiggle)
                elif pos == 4:
                    self.pwm.set_pwm(4, 0, self.pwm4 + wiggle)
                    if self.leftSide_height:
                        self.pwm.set_pwm(5, 0, self.pwm5 - wiggle)
                    else:
                        self.pwm.set_pwm(5, 0, self.pwm5 + wiggle)

    def right_I(self, pos, wiggle, heightAdjust=0):
        # wiggle = -wiggle
        if pos == 0:
            # self.pwm.set_pwm(6,0,self.pwm6)
            if self.rightSide_height:
                self.pwm.set_pwm(7, 0, self.pwm7 + heightAdjust)
            else:
                self.pwm.set_pwm(7, 0, self.pwm7 - heightAdjust)
        else:
            if self.rightSide_direction:
                if pos == 1:
                    self.pwm.set_pwm(6, 0, self.pwm6)
                    if self.rightSide_height:
                        self.pwm.set_pwm(7, 0, self.pwm7 + 3 * self.height_change)
                    else:
                        self.pwm.set_pwm(7, 0, self.pwm7 - 3 * self.height_change)
                elif pos == 2:
                    self.pwm.set_pwm(6, 0, self.pwm6 + wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(7, 0, self.pwm7 - self.height_change)
                    else:
                        self.pwm.set_pwm(7, 0, self.pwm7 + self.height_change)
                elif pos == 3:
                    self.pwm.set_pwm(6, 0, self.pwm6)
                    if self.rightSide_height:
                        self.pwm.set_pwm(7, 0, self.pwm7 - self.height_change)
                    else:
                        self.pwm.set_pwm(7, 0, self.pwm7 + self.height_change)
                elif pos == 4:
                    self.pwm.set_pwm(6, 0, self.pwm6 - wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(7, 0, self.pwm7 - self.height_change)
                    else:
                        self.pwm.set_pwm(7, 0, self.pwm7 + self.height_change)
            else:
                if pos == 1:
                    self.pwm.set_pwm(6, 0, self.pwm6)
                    if self.rightSide_height:
                        self.pwm.set_pwm(7, 0, self.pwm7 + 3 * self.height_change)
                    else:
                        self.pwm.set_pwm(7, 0, self.pwm7 - 3 * self.height_change)
                elif pos == 2:
                    self.pwm.set_pwm(6, 0, self.pwm6 - wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(7, 0, self.pwm7 - self.height_change)
                    else:
                        self.pwm.set_pwm(7, 0, self.pwm7 + self.height_change)
                elif pos == 3:
                    self.pwm.set_pwm(6, 0, self.pwm6)
                    if self.rightSide_height:
                        self.pwm.set_pwm(7, 0, self.pwm7 - self.height_change)
                    else:
                        self.pwm.set_pwm(7, 0, self.pwm7 + self.height_change)
                elif pos == 4:
                    self.pwm.set_pwm(6, 0, self.pwm6 + wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(7, 0, self.pwm7 - self.height_change)
                    else:
                        self.pwm.set_pwm(7, 0, self.pwm7 + self.height_change)

    def right_II(self, pos, wiggle, heightAdjust=0):
        # wiggle = -wiggle
        if pos == 0:
            # self.pwm.set_pwm(8,0,self.pwm8)
            if self.rightSide_height:
                self.pwm.set_pwm(9, 0, self.pwm9 + heightAdjust)
            else:
                self.pwm.set_pwm(9, 0, self.pwm9 - heightAdjust)
        else:
            if self.rightSide_direction:
                if pos == 1:
                    self.pwm.set_pwm(8, 0, self.pwm8)
                    if self.rightSide_height:
                        self.pwm.set_pwm(9, 0, self.pwm9 + 3 * self.height_change)
                    else:
                        self.pwm.set_pwm(9, 0, self.pwm9 - 3 * self.height_change)
                elif pos == 2:
                    self.pwm.set_pwm(8, 0, self.pwm8 + wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(9, 0, self.pwm9 - self.height_change)
                    else:
                        self.pwm.set_pwm(9, 0, self.pwm9 + self.height_change)
                elif pos == 3:
                    self.pwm.set_pwm(8, 0, self.pwm8)
                    if self.rightSide_height:
                        self.pwm.set_pwm(9, 0, self.pwm9 - self.height_change)
                    else:
                        self.pwm.set_pwm(9, 0, self.pwm9 + self.height_change)
                elif pos == 4:
                    self.pwm.set_pwm(8, 0, self.pwm8 - wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(9, 0, self.pwm9 - self.height_change)
                    else:
                        self.pwm.set_pwm(9, 0, self.pwm9 + self.height_change)
            else:
                if pos == 1:
                    self.pwm.set_pwm(8, 0, self.pwm8)
                    if self.rightSide_height:
                        self.pwm.set_pwm(9, 0, self.pwm9 + 3 * self.height_change)
                    else:
                        self.pwm.set_pwm(9, 0, self.pwm9 - 3 * self.height_change)
                elif pos == 2:
                    self.pwm.set_pwm(8, 0, self.pwm8 - wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(9, 0, self.pwm9 - self.height_change)
                    else:
                        self.pwm.set_pwm(9, 0, self.pwm9 + self.height_change)
                elif pos == 3:
                    self.pwm.set_pwm(8, 0, self.pwm8)
                    if self.rightSide_height:
                        self.pwm.set_pwm(9, 0, self.pwm9 - self.height_change)
                    else:
                        self.pwm.set_pwm(9, 0, self.pwm9 + self.height_change)
                elif pos == 4:
                    self.pwm.set_pwm(8, 0, self.pwm8 + wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(9, 0, self.pwm9 - self.height_change)
                    else:
                        self.pwm.set_pwm(9, 0, self.pwm9 + self.height_change)

    def right_III(self, pos, wiggle, heightAdjust=0):
        # wiggle = -wiggle
        if pos == 0:
            # self.pwm.set_pwm(10,0,self.pwm10)
            if self.rightSide_height:
                self.pwm.set_pwm(11, 0, self.pwm11 + heightAdjust)
            else:
                self.pwm.set_pwm(11, 0, self.pwm11 - heightAdjust)
        else:
            if self.rightSide_direction:
                if pos == 1:
                    self.pwm.set_pwm(10, 0, self.pwm10)
                    if self.rightSide_height:
                        self.pwm.set_pwm(11, 0, self.pwm11 + 3 * self.height_change)
                    else:
                        self.pwm.set_pwm(11, 0, self.pwm11 - 3 * self.height_change)
                elif pos == 2:
                    self.pwm.set_pwm(10, 0, self.pwm10 + wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(11, 0, self.pwm11 - self.height_change)
                    else:
                        self.pwm.set_pwm(11, 0, self.pwm11 + self.height_change)
                elif pos == 3:
                    self.pwm.set_pwm(10, 0, pwm10)
                    if self.rightSide_height:
                        self.pwm.set_pwm(11, 0, self.pwm11 - self.height_change)
                    else:
                        self.pwm.set_pwm(11, 0, self.pwm11 + self.height_change)
                elif pos == 4:
                    self.pwm.set_pwm(10, 0, self.pwm10 - wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(11, 0, self.pwm11 - self.height_change)
                    else:
                        self.pwm.set_pwm(11, 0, self.pwm11 + self.height_change)
            else:
                if pos == 1:
                    self.pwm.set_pwm(10, 0, self.pwm10)
                    if self.rightSide_height:
                        self.pwm.set_pwm(11, 0, self.pwm11 + 3 * self.height_change)
                    else:
                        self.pwm.set_pwm(11, 0, self.pwm11 - 3 * self.height_change)
                elif pos == 2:
                    self.pwm.set_pwm(10, 0, self.pwm10 - wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(11, 0, self.pwm11 - self.height_change)
                    else:
                        self.pwm.set_pwm(11, 0, self.pwm11 + self.height_change)
                elif pos == 3:
                    self.pwm.set_pwm(10, 0, self.pwm10)
                    if self.rightSide_height:
                        self.pwm.set_pwm(11, 0, self.pwm11 - self.height_change)
                    else:
                        self.pwm.set_pwm(11, 0, self.pwm11 + self.height_change)
                elif pos == 4:
                    self.pwm.set_pwm(10, 0, self.pwm10 + wiggle)
                    if self.rightSide_height:
                        self.pwm.set_pwm(11, 0, self.pwm11 - self.height_change)
                    else:
                        self.pwm.set_pwm(11, 0, self.pwm11 + self.height_change)

    def move(self, step_input, speed, command):
        step_I = step_input
        step_II = step_input + 2

        if step_II > 4:
            step_II = step_II - 4
        if speed == 0:
            return

        if command == 'no':
            self.right_I(step_I, speed, 0)
            self.left_II(step_I, speed, 0)
            self.right_III(step_I, speed, 0)

            self.left_I(step_II, speed, 0)
            self.right_II(step_II, speed, 0)
            self.left_III(step_II, speed, 0)
        elif command == 'left':
            self.right_I(step_I, speed, 0)
            self.left_II(step_I, -speed, 0)
            self.right_III(step_I, speed, 0)

            self.left_I(step_II, -speed, 0)
            self.right_II(step_II, speed, 0)
            self.left_III(step_II, -speed, 0)
        elif command == 'right':
            self.right_I(step_I, -speed, 0)
            self.left_II(step_I, speed, 0)
            self.right_III(step_I, -speed, 0)

            self.left_I(step_II, speed, 0)
            self.right_II(step_II, -speed, 0)
            self.left_III(step_II, speed, 0)


    '''
    Default standing position
    '''
    def stand(self):
        self.pwm.set_pwm(0, 0, 300)
        self.pwm.set_pwm(1, 0, 300)
        self.pwm.set_pwm(2, 0, 300)
        self.pwm.set_pwm(3, 0, 300)
        self.pwm.set_pwm(4, 0, 300)
        self.pwm.set_pwm(5, 0, 300)
        self.pwm.set_pwm(6, 0, 300)
        self.pwm.set_pwm(7, 0, 300)
        self.pwm.set_pwm(8, 0, 300)
        self.pwm.set_pwm(9, 0, 300)
        self.pwm.set_pwm(10, 0, 300)
        self.pwm.set_pwm(11, 0, 300)

    '''
    Makes servo movement smooth
    '''

    def dove_Left_I(self, horizontal, vertical):
        if self.leftSide_direction:
            self.pwm.set_pwm(0, 0, self.pwm0 + horizontal)
        else:
            self.pwm.set_pwm(0, 0, self.pwm0 - horizontal)

        if self.leftSide_height:
            self.pwm.set_pwm(1, 0, self.pwm1 + vertical)
        else:
            self.pwm.set_pwm(1, 0, self.pwm1 - vertical)

    def dove_Left_II(self, horizontal, vertical):
        if self.leftSide_direction:
            self.pwm.set_pwm(2, 0, self.pwm2 + horizontal)
        else:
            self.pwm.set_pwm(2, 0, self.pwm2 - horizontal)

        if self.leftSide_height:
            self.pwm.set_pwm(3, 0, self.pwm3 + vertical)
        else:
            self.pwm.set_pwm(3, 0, self.pwm3 - vertical)

    def dove_Left_III(self, horizontal, vertical):
        if self.leftSide_direction:
            self.pwm.set_pwm(4, 0, self.pwm4 + horizontal)
        else:
            self.pwm.set_pwm(4, 0, self.pwm4 - horizontal)

        if self.leftSide_height:
            self.pwm.set_pwm(5, 0, self.pwm5 + vertical)
        else:
            self.pwm.set_pwm(5, 0, self.pwm5 - vertical)

    def dove_Right_I(self, horizontal, vertical):
        if self.rightSide_direction:
            self.pwm.set_pwm(6, 0, self.pwm6 + horizontal)
        else:
            self.pwm.set_pwm(6, 0, self.pwm6 - horizontal)

        if self.rightSide_height:
            self.pwm.set_pwm(7, 0, self.pwm7 + vertical)
        else:
            self.pwm.set_pwm(7, 0, self.pwm7 - vertical)

    def dove_Right_II(self, horizontal, vertical):
        if self.rightSide_direction:
            self.pwm.set_pwm(8, 0, self.pwm8 + horizontal)
        else:
            self.pwm.set_pwm(8, 0, self.pwm8 - horizontal)

        if self.rightSide_height:
            self.pwm.set_pwm(9, 0, self.pwm9 + vertical)
        else:
            self.pwm.set_pwm(9, 0, self.pwm9 - vertical)

    def dove_Right_III(self, horizontal, vertical):
        if self.rightSide_direction:
            self.pwm.set_pwm(10, 0, self.pwm10 + horizontal)
        else:
            self.pwm.set_pwm(10, 0, self.pwm10 - horizontal)

        if self.rightSide_height:
            self.pwm.set_pwm(11, 0, self.pwm11 + vertical)
        else:
            self.pwm.set_pwm(11, 0, self.pwm11 - vertical)


    def dove(self, step_input, speed, timeLast, dpi, command):
        step_I = step_input
        step_II = step_input + 2

        if step_II > 4:
            step_II = step_II - 4

        if speed > 0:
            if step_input == 1:
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(-speed_I, 3 * speed_II)
                        self.dove_Right_II(-speed_I, 3 * speed_II)
                        self.dove_Left_III(-speed_I, 3 * speed_II)

                        self.dove_Right_I(speed_I, -10)
                        self.dove_Left_II(speed_I, -10)
                        self.dove_Right_III(speed_I, -10)
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if command == 'left':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(speed_I, 3 * speed_II)
                        self.dove_Right_II(-speed_I, 3 * speed_II)
                        self.dove_Left_III(speed_I, 3 * speed_II)

                        self.dove_Right_I(speed_I, -10)
                        self.dove_Left_II(-speed_I, -10)
                        self.dove_Right_III(speed_I, -10)
                        time.sleep(timeLast / dpi)
                    elif command == 'right':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(-speed_I, 3 * speed_II)
                        self.dove_Right_II(speed_I, 3 * speed_II)
                        self.dove_Left_III(-speed_I, 3 * speed_II)

                        self.dove_Right_I(-speed_I, -10)
                        self.dove_Left_II(speed_I, -10)
                        self.dove_Right_III(-speed_I, -10)
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if self.move_stu == 0 and command == 'no':
                        break

            elif step_input == 2:
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(speed_II, 3 * (speed - speed_II))
                        self.dove_Right_II(speed_II, 3 * (speed - speed_II))
                        self.dove_Left_III(speed_II, 3 * (speed - speed_II))

                        self.dove_Right_I(-speed_II, -10)
                        self.dove_Left_II(-speed_II, -10)
                        self.dove_Right_III(-speed_II, -10)
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if command == 'left':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(-speed_II, 3 * (speed - speed_II))
                        self.dove_Right_II(speed_II, 3 * (speed - speed_II))
                        self.dove_Left_III(-speed_II, 3 * (speed - speed_II))

                        self.dove_Right_I(-speed_II, -10)
                        self.dove_Left_II(speed_II, -10)
                        self.dove_Right_III(-speed_II, -10)
                        time.sleep(timeLast / dpi)
                    elif command == 'right':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(speed_II, 3 * (speed - speed_II))
                        self.dove_Right_II(-speed_II, 3 * (speed - speed_II))
                        self.dove_Left_III(speed_II, 3 * (speed - speed_II))

                        self.dove_Right_I(speed_II, -10)
                        self.dove_Left_II(-speed_II, -10)
                        self.dove_Right_III(speed_II, -10)
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if self.move_stu == 0 and command == 'no':
                        break
            elif step_input == 3:
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(speed_I, -10)
                        self.dove_Right_II(speed_I, -10)
                        self.dove_Left_III(speed_I, -10)

                        self.dove_Right_I(-speed_I, 3 * speed_II)
                        self.dove_Left_II(-speed_I, 3 * speed_II)
                        self.dove_Right_III(-speed_I, 3 * speed_II)
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if command == 'left':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(-speed_I, -10)
                        self.dove_Right_II(speed_I, -10)
                        self.dove_Left_III(-speed_I, -10)

                        self.dove_Right_I(-speed_I, 3 * speed_II)
                        self.dove_Left_II(speed_I, 3 * speed_II)
                        self.dove_Right_III(-speed_I, 3 * speed_II)
                        time.sleep(timeLast / dpi)
                    elif command == 'right':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(speed_I, -10)
                        self.dove_Right_II(-speed_I, -10)
                        self.dove_Left_III(speed_I, -10)

                        self.dove_Right_I(speed_I, 3 * speed_II)
                        self.dove_Left_II(-speed_I, 3 * speed_II)
                        self.dove_Right_III(speed_I, 3 * speed_II)
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if self.move_stu == 0 and command == 'no':
                        break
            elif step_input == 4:
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(-speed_II, -10)
                        self.dove_Right_II(-speed_II, -10)
                        self.dove_Left_III(-speed_II, -10)

                        self.dove_Right_I(speed_II, 3 * (speed - speed_II))
                        self.dove_Left_II(speed_II, 3 * (speed - speed_II))
                        self.dove_Right_III(speed_II, 3 * (speed - speed_II))
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if command == 'left':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(speed_II, -10)
                        self.dove_Right_II(-speed_II, -10)
                        self.dove_Left_III(speed_II, -10)

                        self.dove_Right_I(speed_II, 3 * (speed - speed_II))
                        self.dove_Left_II(-speed_II, 3 * (speed - speed_II))
                        self.dove_Right_III(speed_II, 3 * (speed - speed_II))
                        time.sleep(timeLast / dpi)
                    elif command == 'right':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(-speed_II, -10)
                        self.dove_Right_II(speed_II, -10)
                        self.dove_Left_III(-speed_II, -10)

                        self.dove_Right_I(-speed_II, 3 * (speed - speed_II))
                        self.dove_Left_II(speed_II, 3 * (speed - speed_II))
                        self.dove_Right_III(-speed_II, 3 * (speed - speed_II))
                        time.sleep(timeLast / dpi)
                    else:
                        pass

                    if self.move_stu == 0 and command == 'no':
                        break
        else:
            speed = -speed
            if step_input == 1:
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(speed_I, 3 * speed_II)
                        self.dove_Right_II(speed_I, 3 * speed_II)
                        self.dove_Left_III(speed_I, 3 * speed_II)

                        self.dove_Right_I(-speed_I, -10)
                        self.dove_Left_II(-speed_I, -10)
                        self.dove_Right_III(-speed_I, -10)
                        time.sleep(timeLast / dpi)
                    else:
                        pass
            elif step_input == 2:
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(-speed_II, 3 * (speed - speed_II))
                        self.dove_Right_II(-speed_II, 3 * (speed - speed_II))
                        self.dove_Left_III(-speed_II, 3 * (speed - speed_II))

                        self.dove_Right_I(speed_II, -10)
                        self.dove_Left_II(speed_II, -10)
                        self.dove_Right_III(speed_II, -10)
                        time.sleep(timeLast / dpi)
                    else:
                        pass
            elif step_input == 3:
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(-speed_I, -10)
                        self.dove_Right_II(-speed_I, -10)
                        self.dove_Left_III(-speed_I, -10)

                        self.dove_Right_I(speed_I, 3 * speed_II)
                        self.dove_Left_II(speed_I, 3 * speed_II)
                        self.dove_Right_III(speed_I, 3 * speed_II)
                        time.sleep(timeLast / dpi)
                    else:
                        pass
            elif step_input == 4:
                for speed_I in range(0, (speed + int(speed / dpi)), int(speed / dpi)):
                    if self.move_stu and command == 'no':
                        speed_II = speed_I
                        speed_I = speed - speed_I
                        self.dove_Left_I(speed_II, -10)
                        self.dove_Right_II(speed_II, -10)
                        self.dove_Left_III(speed_II, -10)

                        self.dove_Right_I(-speed_II, 3 * (speed - speed_II))
                        self.dove_Left_II(-speed_II, 3 * (speed - speed_II))
                        self.dove_Right_III(-speed_II, 3 * (speed - speed_II))
                        time.sleep(timeLast / dpi)
                    else:
                        pass

    '''
    Control head servos
    '''

    def look_up(self, wiggle=30):
        if self.Up_Down_direction:
            self.Up_Down_input += wiggle
            self.Up_Down_input = self.ctrl_range(self.Up_Down_input, self.Up_Down_Max, self.Up_Down_Min)
        else:
            self.Up_Down_input -= wiggle
            self.Up_Down_input = self.ctrl_range(self.Up_Down_input, self.Up_Down_Max, self.Up_Down_Min)
        self.pwm.set_pwm(13, 0, self.Up_Down_input)

    def look_down(self, wiggle=30):
        if self.Up_Down_direction:
            self.Up_Down_input -= wiggle
            self.Up_Down_input = self.ctrl_range(self.Up_Down_input, self.Up_Down_Max, self.Up_Down_Min)
        else:
            self.Up_Down_input += wiggle
            self.Up_Down_input = self.ctrl_range(self.Up_Down_input, self.Up_Down_Max, self.Up_Down_Min)
        self.pwm.set_pwm(13, 0, self.Up_Down_input)

    def look_left(self, wiggle=30):
        if self.Left_Right_direction:
            self.Left_Right_input += wiggle
            self.Left_Right_input = self.ctrl_range(self.Left_Right_input, self.Left_Right_Max, self.Left_Right_Min)
        else:
            self.Left_Right_input -= wiggle
            self.Left_Right_input = self.ctrl_range(self.Left_Right_input, self.Left_Right_Max, self.Left_Right_Min)
        self.pwm.set_pwm(12, 0, self.Left_Right_input)

    def look_right(self, wiggle=30):
        if self.Left_Right_direction:
            self.Left_Right_input -= wiggle
            self.Left_Right_input = self.ctrl_range(self.Left_Right_input, self.Left_Right_Max, self.Left_Right_Min)
        else:
            self.Left_Right_input += wiggle
            self.Left_Right_input = self.ctrl_range(self.Left_Right_input, self.Left_Right_Max, self.Left_Right_Min)
        self.pwm.set_pwm(12, 0, self.Left_Right_input)

    def look_home(self):
        self.pwm.set_pwm(13, 0, 300)
        self.pwm.set_pwm(12, 0, 300)
        self.Left_Right_input = 300
        self.Up_Down_input = 300

    '''
    Reset servos
    '''

    def clean_all(self):
        self.pwm.set_all_pwm(0, 0)

    def destroy(self):
        self.clean_all()

    def move_thread(self):
        stand_stu = 1
        if not self.balanceMode:
            if self.direction_command == 'forward' and self.turn_command == 'no':
                if self.smoothMode:
                    self.dove(self.step_set, 35, 0.001, self.DPI, 'no')
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

                else:
                    self.move(self.step_set, 35, 'no')
                    time.sleep(0.1)
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

            elif self.direction_command == 'backward' and self.turn_command == 'no':
                if self.smoothMode:
                    self.dove(self.step_set, -35, 0.001, self.DPI, 'no')
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

                else:
                    self.move(self.step_set, -35, 'no')
                    time.sleep(0.1)
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

            else:
                pass

            if self.turn_command != 'no':
                if self.smoothMode:
                    self.dove(self.step_set, 35, 0.001, self.DPI, self.turn_command)
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

                else:
                    self.move(self.step_set, 35, self.turn_command)
                    time.sleep(0.1)
                    self.step_set += 1
                    if self.step_set == 5:
                        self.step_set = 1

            else:
                pass

            if self.turn_command == 'no' and self.direction_command == 'stand':
                self.stand()
                self.step_set = 1
            pass
        else:
            pass


    def mpu6050Test(self):
        while 1:
            self.accelerometer_data = self.sensor.get_accel_data()
            print('X=%f,Y=%f,Z=%f' % (self.accelerometer_data['x'], self.accelerometer_data['y'],
                                      self.accelerometer_data['x']))
            time.sleep(0.3)


class RobotM(threading.Thread):
    def __init__(self, body, *args, **kwargs):
        super(RobotM, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()
        self.body = body

    def pause(self):
        # print('......................pause..........................')
        self.__flag.clear()

    def resume(self):
        self.__flag.set()

    def run(self):
        while 1:
            self.__flag.wait()
            self.body.move_thread()
            pass

'''
Issue commands to body
'''

def commandInput(command_input, body):

    if 'forward' == command_input:
        body.direction_command = 'forward'
        rm.resume()

    elif 'backward' == command_input:
        body.direction_command = 'backward'
        rm.resume()

    elif 'stand' in command_input:
        body.direction_command = 'stand'
        body.turn_command = 'no'
        rm.pause()

    elif 'left' == command_input:
        body.turn_command = 'left'
        rm.resume()

    elif 'right' == command_input:
        body.turn_command = 'right'
        rm.resume()

    elif 'no' in command_input:
        body.turn_command = 'no'
        rm.pause()

    elif 'automaticOff' == command_input:
        body.smoothMode = 0
        body.balanceMode = 0
        rm.pause()

    elif 'automatic' == command_input:
        rm.resume()
        body.smoothMode = 1



'''
Instantiation
'''
# Hexapod body
Hexapod = Body('Hexapod')
Hexapod.init_all()

# Thread executing movement
rm = RobotM(Hexapod)
rm.start()
rm.pause()

print(f'Body initialization successful, Body: {str(rm.body.name)}')


'''
Testing
'''
if __name__ == '__main__':
    
    Hexapod.step_set = 1
    Hexapod.move_stu = 1
    try:

        
        while 1:
            Hexapod.move(Hexapod.step_set, 35, 'no')
            Hexapod.step_set += 1
            if Hexapod.step_set > 4:
                Hexapod.step_set = 1
            time.sleep(0.12)
        

        '''
        while 1:
            Hexapod.dove(1,-35,0.01,17,'no')
            Hexapod.dove(2,-35,0.01,17,'no')
            Hexapod.dove(3,-35,0.01,17,'no')
            Hexapod.dove(4,-35,0.01,17,'no')
        '''

        # Hexapod.mpu6050Test()
        # print(Hexapod.sensor.get_temp())
        '''

        for i in range(0, 4):
            Hexapod.look_left()
            time.sleep(1)
        for i in range(0, 8):
            Hexapod.look_right()
            time.sleep(1)
        time.sleep(1)
        Hexapod.look_home()


        '''
        ''''''
        # Hexapod.pwm.set_all_pwm(0,0)
        # Hexapod.pwm.set_all_pwm(0, 300)
        # time.sleep(10)

    except KeyboardInterrupt:
        Hexapod.pwm.set_all_pwm(0, 300)
        time.sleep(1)
        Hexapod.clean_all()
