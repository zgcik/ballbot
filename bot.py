import time
import serial
import numpy as np

from camera import Camera

class Bot:
    def __init__(self):
        # calibration
        self.wheel = 0.05
        self.baseline = 0.0185

        # init robot pose
        self.pose = [0.0, 0.0, 0.0]

        # init cam
        self.cam = Camera(device=0)

        # setting up arduino communication
        self.arduino_port = '/dev/ttyACM0'
        self.baud_rate = 9600

        self.setup_arduino()

    def setup_arduino(self):
        with serial.Serial(self.arduino_port, self.baud_rate, timeout=1) as ser:
                # initialise arduino
                time.sleep(2)
                ser.flushInput()
                ser.flushOutput()

    def send_command(self, command):
        try:
            with serial.Serial(self.arduino_port, self.baud_rate, timeout=1) as ser:
                # send command
                ser.write(f'{command}\n'.encode())

                # getting response
                if ser.in_waiting > 0:
                    response = ser.readline().decode('utf-8').strip()
                    return response
                else:
                    print('no response from arduino')
                    return None
        except serial.SerialException as e:
            print(f'error in serial communication: {e}')
            return None    
    
    def update_pose(self, response):
        if response is None: return

        # obtaining arduino response values
        left_val, right_val = response.split(',')
        left_revs = float(left_val.split(':')[1])
        right_revs = float(right_val.split(':')[1])

        delta_left = left_revs * (2 * np.pi)
        delta_right = right_revs * (2 * np.pi)

        # change in angle and distance
        ang = (delta_right - delta_left) / self.baseline
        distance = (delta_left + delta_right) / 2

        # update theta
        self.pose[2] = (self.pose[2] + ang) % (2 * np.pi)

        # update pose
        if distance > 0:
            self.pose[0] += distance * np.cos(self.pose[2])
            self.pose[1] += distance * np.sin(self.pose[2])

    def __calc_revs__(self, dis):
        return dis / self.wheel

    def drive(self, dis):
        response = self.send_command(f'$drive: {self.__calc_revs__(dis)} rev')
        self.update_pose(response)

    def rotate(self, ang):
        response = self.send_command(f'$turn: {ang} rad')
        self.update_pose(response)

    def flip(self):
        self.send_command(f'$flip: theta 180 DT 60')