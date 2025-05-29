import serial
from serial import Serial
from serial.tools import list_ports
import time
import numpy as np
import struct

class BLEInterface:

    def __init__(self):
        board_name = "Nano 33 BLE"

        arduino_port = self.find_arduino(board_name)
        self.arduino = serial.Serial(arduino_port, baudrate=115200, timeout=0.1)
        time.sleep(2)
        self.arduino.flush()

    def find_arduino(self, device_name):
        ports = list(list_ports.comports())
        for p in ports:
            if p.description == device_name:
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

class LiDARInterface:

    def __init__(self):
        lidar_name = "CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller"
        
        lidar_port = self.find_lidar(lidar_name)
        self.lidar = serial.Serial(lidar_port, baudrate=230400, timeout=0.1)


    def find_lidar(self, device_name):
        ports = list(list_ports.comports())
        for p in ports:
            if p.description == device_name:
                return p.device

    def read_raw_data(self):

        try:
            attempts = 0
            HEADER = None
            while attempts <= 100:
                packet_1 = self.lidar.read(1)
                packet_2 = self.lidar.read(1)
                HEADER = packet_1 + packet_2

                if HEADER == b'\x54\x2C':
                    packet = HEADER + self.lidar.read(45)

                    speed_raw = float(struct.unpack_from('<H', packet, 2)[0])
                    start_angle_raw = float(struct.unpack_from('<H', packet, 4)[0]*np.pi/180/100)

                    distance_data_raw, intensity_data_raw = [], []
                    for i in range(6, 42, 3):
                        distance = float(struct.unpack_from('<H', packet, i)[0]/1000)
                        intensity = float(struct.unpack_from('<B', packet, i + 2)[0]/1000)
                        distance_data_raw.append(distance)
                        intensity_data_raw.append(intensity)

                    end_angle_raw = float(struct.unpack_from('<H', packet, 42)[0]*np.pi/180/100)
                    time_raw = int(struct.unpack_from('<H', packet, 44)[0])

                    return speed_raw, start_angle_raw, distance_data_raw, intensity_data_raw, end_angle_raw, time_raw
                attempts += 1
            return None
        except Exception as e:
            print(f"Error in reading: {e}")

if __name__ == '__main__':
    arduino_ble = BLEInterface()
    ldrobot = LiDARInterface()