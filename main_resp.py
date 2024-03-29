from device import DWM1001
import sys


if len(sys.argv) != 2:
    port = '/dev/ttyACM0'
else:
    port = sys.argv[1]

device = DWM1001(port)

while True:
    output = device.get_output()
    if output != '':
        with open('led_color', 'w') as led_color:
            print('Led color :', output.strip())
            led_color.write(output)
