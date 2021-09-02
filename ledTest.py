# Test program for MARS Rover Smart RGB LEDs

from __future__ import print_function
import rover, time

numLEDs = rover.numPixels
RED = rover.fromRGB(255,0,0)
ORANGE = rover.fromRGB(255,255,0)
GREEN = rover.fromRGB(0,255,0)
BLUE = rover.fromRGB(0,0,255)
BLACK = rover.fromRGB(0,0,0)
WHITE = rover.fromRGB(255,255,255)

rover.init(40)

try:
    while True:
        for i in range(numLEDs):
            rover.setPixel(i, RED)
        rover.show()
        time.sleep(1)
        for i in range(numLEDs):
            rover.setPixel(i, GREEN)
        rover.show()
        time.sleep(1)
        for i in range(numLEDs):
            rover.setPixel(i, BLUE)
        rover.show()
        time.sleep(1)
        for i in range(numLEDs):
            rover.setPixel(i, WHITE)
        rover.show()
        time.sleep(1)

except KeyboardInterrupt:
    print

try:
    rover.cleanup()
except Exception as e:
    print (e)
