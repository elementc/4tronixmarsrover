#!/usr/bin/python
# servoTest.py

from __future__ import print_function
import rover

# Define variables for servo and degrees
servo = 0 # currently active servo (0 to 15)
degrees = 0 # Current angle of servo. 0 degrees is centre (-90 to +90)

#======================================================================
# Reading single character by forcing stdin to raw mode
import sys
import tty
import termios

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)  # 16=Up, 17=Down, 18=Right, 19=Left arrows

# End of single character reading
#======================================================================


rover.init(0)

print ("Use Arrows or W-Up, Z-Down, A-Left, S-Right Space=Centre, ^C=Exit:")

try:
    servo = 0
    while True:
        key = readkey()
        if key == ' ':
            degrees = 0
            rover.setServo(servo, degrees)
            print ('Servo ', servo, ': Centre',sep='')

        if key == 'x' or key == '.':
            rover.stopServos()
            print ('Servo ', servo, ': Stop',sep='')

        elif key == 'w' or ord(key) == 16:
            servo += 1
            servo %= 16
            print ('Servo ', servo, ': Selected',sep='')

        elif key == 'z' or ord(key) == 17:
            servo -= 1
            servo %= 16
            print ('Servo ', servo, ': Selected',sep='')

        elif key == 'a' or ord(key) == 19:
            degrees -= 5
            if (degrees < -90):
                degrees = -90
            rover.setServo(servo, degrees)
            print ('Servo ', servo, ': Value:', degrees, sep='')

        elif key == 's' or ord(key) == 18:
            degrees += 5
            if (degrees > 90):
                degrees = 90
            rover.setServo(servo, degrees)
            print ('Servo ', servo, ': Value:', degrees, sep='')

        elif ord(key) == 3:
            break

except KeyboardInterrupt:
    print()

finally:
    rover.cleanup()
