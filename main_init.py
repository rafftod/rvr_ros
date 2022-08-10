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
    return colors
    

if len(sys.argv) != 2:
    port = '/dev/ttyACM1'
else:
    port = sys.argv[1]

device = DWM1001(port)
start_time = time.time()
last_update = start_time

while True:
    now = time.time()
    if now - last_update > UPDATE_INTERVAL:
        with open('floor_color') as floor_color:
            try:
                red, green, blue = floor_color.readline().strip().split()
                colors = [int(red), int(green), int(blue)]
                colors = saturate([red, green, blue])
                print(colors)
                device.set_color(colors[0], colors[1], colors[2])
                device.send_transmission()
                
            except Exception as e:
                pass
            last_update = now