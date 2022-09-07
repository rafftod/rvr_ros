from device import DWM1001
import time
import sys

UPDATE_INTERVAL = 0.5

def saturate(color):
    max_comp = max(color)
    if max_comp == 0:
        return [0, 0, 0]
    factor = 255 / max_comp
    for i in range(len(color)):
        color[i] = int(color[i] * factor)
        if color[i] > 255:
            color[i] = 255
    

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
                r, g, b = floor_color.readline().strip().split()
                color = [int(r), int(g), int(b)]
            except Exception as e:
                color = None

            if color is not None:
                saturate(color)
                print('Floor color :', color)
                device.set_color(color)
                device.send_transmission()
                
            last_update = now
