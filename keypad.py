# M.A.R.S. Rover KeyPad Test

from __future__ import print_function
import rover

rover.init(0)
count = 0

print ("Touch keys on keypad")
print ("Hit ctrl-C when done")

try:
    while True:
        key = rover.getKey()
        count += 1
        print (count, "Key: ", key)

except KeyboardInterrupt:
    print()

finally:
    rover.cleanup()
