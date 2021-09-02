# Calibrates and saves Servo Offsets on MARS Rover

from __future__ import print_function
import rover,time

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

print ("Calibrate the servos on M.A.R.S. Rover")
print ("Select Servo to claibrate with '1', '2', '3', '4' or '5'")
print ("Use the left and right arrow keys to straighten the servo")
print ("Press 's' to save the calibration data in the EEROM")
print ("Press Ctrl-C to exit without saving")
print ()

rover.init(40)

red = rover.fromRGB(255,0,0)
green = rover.fromRGB(0,255,0)
blue = rover.fromRGB(0,0,255)

rover.setColor(red)
rover.show()
print ('Existing servo offsets')
print (rover.offsets)
print ()

try:
    servo = 9
    print ('Servo: Front Left: Offset:', rover.offsets[servo], sep='')
    rover.setColor(red)
    rover.setPixel(1, green)
    rover.show()
    while True:
        key = readkey()
        if key == '1':
            servo = 9
            print ('Servo: Front Left: Offset:', rover.offsets[servo], sep='')
            rover.setColor(red)
            rover.setPixel(1, green)
            rover.show()
        elif key == '2':
            servo = 11
            print ('Servo: Rear Left: Offset:', rover.offsets[servo], sep='')
            rover.setColor(red)
            rover.setPixel(0, green)
            rover.show()
        elif key == '3':
            servo = 15
            print ('Servo: Front Right: Offset:', rover.offsets[servo], sep='')
            rover.setColor(red)
            rover.setPixel(2, green)
            rover.show()
        elif key == '4':
            servo = 13
            print ('Servo: Rear Right: Offset:', rover.offsets[servo], sep='')
            rover.setColor(red)
            rover.setPixel(3, green)
            rover.show()
        elif key == '5':
            servo = 0
            print ('Servo: Mast: Offset:', rover.offsets[servo], sep='')
            rover.setColor(red)
            rover.setPixel(1, blue)
            rover.setPixel(2, blue)
            rover.show()
        elif key == 'x' or key == '.':
            rover.stopServos()
            print ('Servo ', servo, ': Stop',sep='')
        elif ord(key) == 19:
            rover.offsets[servo] -= 1
            rover.setServo(servo, 0)
            print ('Servo ', servo, ': Offset:', rover.offsets[servo], sep='')
        elif ord(key) == 18:
            rover.offsets[servo] += 1
            rover.setServo(servo, 0)
            print ('Servo ', servo, ': Offset:', rover.offsets[servo], sep='')
        elif key == 's':
            print ('Saving servo offsets')
            rover.saveOffsets()
            break
        elif ord(key) == 3:
            break

except KeyboardInterrupt:
    print()

finally:
    rover.loadOffsets()
    print ()
    print ('New servo offsets')
    print (rover.offsets)
    print ()
    rover.cleanup()
