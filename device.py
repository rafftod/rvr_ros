import serial

DEVICE_PORT = '/dev/ttyACM0'
DEVICE_BAUDRATE = 115200
DEVICE_BYTESIZE = serial.EIGHTBITS
DEVICE_PARITY = serial.PARITY_NONE
DEVICE_STOPBITS = serial.STOPBITS_ONE
DEVICE_TIMEOUT = 1

class DWM1001:
    def __init__(self, port=None):
        try:
            device = serial.Serial()
            device.port = DEVICE_PORT if port is None else port
            device.baudrate = DEVICE_BAUDRATE
            device.bytesize = DEVICE_BYTESIZE
            device.parity = DEVICE_PARITY
            device.stopbits = DEVICE_STOPBITS
            device.timeout = DEVICE_TIMEOUT
            device.write_timeout = 0
            device.open()
            print("Device successfully connected")
            self.device = device

        except Exception as e:
            print(e)
            print("Could not connect to device on port '" + DEVICE_PORT + "'")
            quit()
    
    def close(self):
        self.device.close()
        print("Connection with the device closed")
    
    def get_output(self):
        return str(self.device.readline(), 'utf-8')

    def show_output(self):
        while True:
            output = self.get_output()
            if output != '':
                print(output, end='')
    
    def write(self, string):
        self.device.write(bytes(string, encoding='utf-8'))
    
    def set_color(self, color):
        self.write('R' + str(color[0]) + 'F')
        self.write('G' + str(color[1]) + 'F')
        self.write('B' + str(color[2]) + 'F')

    def send_transmission(self):
        self.write('S')
