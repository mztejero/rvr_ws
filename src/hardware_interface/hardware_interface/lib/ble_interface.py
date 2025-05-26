import serial
from serial import Serial
from serial.tools import list_ports
import time
import numpy as np

class BLEInterface:

    def __init__(self):
        self.board_name = "Nano 33 BLE"

        arduino_port = self.find_arduino()
        self.arduino = serial.Serial(arduino_port, baudrate=115200, timeout=0.1)
        time.sleep(2)
        self.arduino.flush()

    def find_arduino(self):
        ports = list(list_ports.comports())
        for p in ports:
            if p.description == self.board_name:
                return p.device

    def read_serial(self):
        mag = a = w = None
        try:
            read_msg = 'AAA\n'
            self.arduino.write(read_msg.encode('utf-8'))
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode('utf-8').strip()
                if line:
                    line = line.strip()
                    data = line.split(':')
                    if len(data) == 19:
                        if read_msg[0] == "A":
                            mag = np.array([float(data[2]), float(data[4]), float(data[6])])
                        if read_msg[1] == "A":
                            a = np.array([float(data[8]), float(data[10]), float(data[12])])
                        if read_msg[2] == "A":
                            w = np.array([float(data[14]), float(data[16]), float(data[18])])
                        if mag is not None and a is not None and w is not None:
                            return mag, a, w
                
        except Exception as e:
            print(f"Error in reading: {e}")

if __name__ == '__main__':
    arduino_ble = BLEInterface()