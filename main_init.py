from device import DWM1001
import time
import sys

UPDATE_INTERVAL = 0.5

if len(sys.argv) != 2:
    raise ValueError("Port not given")

device = DWM1001(sys.argv[1])
start_time = time.time()
last_update = start_time

while True:
    now = time.time()
    if now - last_update > UPDATE_INTERVAL:
        with open('floor_color') as floor_color:
            red, green, blue = floor_color.readline().strip().split()
            device.set_color(red, green, blue)
            device.send_transmission()
            last_update = now