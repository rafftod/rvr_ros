from device import DWM1001
import sys

if len(sys.argv) != 2:
    raise ValueError("Port not given")

device = DWM1001(sys.argv[1])

while True:
    output = device.get_output()
    if output != '':
        with open('led_color', 'w') as led_color:
            print(output)
            led_color.write(output + ' ')