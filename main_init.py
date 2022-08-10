from device import DWM1001
import time
import sys

UPDATE_INTERVAL = 0.5

def saturate(colors):
    max_color = max(colors)
    factor = 255 / max_color
    for i in range(len(colors)):
        colors[i] = int(colors[i] * factor)
        if colors[i] > 255:
            colors[i] = 255

if len(sys.argv) != 2:
    raise ValueError("Port not given")

device = DWM1001(sys.argv[1])
start_time = time.time()
last_update = start_time

while True:
    now = time.time()
    if now - last_update > UPDATE_INTERVAL:
        with open('floor_color') as floor_color:
            try:
                red, green, blue = floor_color.readline().strip().split()
                colors = [red, green, blue]
                saturate(colors)
                device.set_color(colors[0], colors[1], colors[2])
                device.send_transmission()
                
            except Exception as e:
                pass
            last_update = now