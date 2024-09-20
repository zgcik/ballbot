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
        self.pose = [0, 0, 0]

        # init cam
        self.cam = Camera(device=0)

        # setting up arduino communication
        self.arduino_port = '/dev/ttyACM0'
        self.baud_rate = 9600

    def send_command(self, command):
        try:
            with serial.Serial(self.arduino_port, self.baud_rate, timeout=1) as ser:
                # initialise arduino (may be placed in instantiation of bot object)
                time.sleep(2)
                ser.flushInput()
                ser.flushOutput()

                # send command
                ser.write(f'{command}\n'.encode())

                if ser.in_waiting > 0:
                    response = ser.readline().decode().strip()
                    # TODO: update pose

                    return response
                else:
                    print('no response from arduino')
                    return None
        except serial.SerialException as e:
            print(f'error in serial communication: {e}')
            return None    
    
    def update_pose(self, dis, ang):
        if dis > 0:
            self.pose[0] += dis * np.cos(self.pose[2])
            self.pose[1] += dis * np.sin(self.pose[2])
        
        self.pose[2] = (self.pose[2] + ang) % (2 * np.pi)

    def __calc_revs__(self, dis):
        return dis / self.wheel

    def drive(self, dis):
        revs = self.__calc_revs__(dis)
        self.send_command(f'$Drive: {revs} Rev')

    def rotate(self, ang):
        pass