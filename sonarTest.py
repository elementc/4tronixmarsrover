# Basic test for sonar on MARS ROver mast

from __future__ import print_function
import rover,time

rover.init(0)

try:
    while True:
        dist = rover.getDistance()
        print ("Distance: ", (int(dist * 10)) / 10.0)
        time.sleep(1)

except KeyboardInterrupt:
    print()

finally:
    rover.cleanup()
