import time
import serial
import numpy as np

from camera import Camera

class Bot:
    def __init__(self):
        # calibration
        self.wheel = 0.05415049396898059
        self.baseline = 0.023

        # init robot state
        self.state = [0.0, 0.0, 0.0]

        # init cam
        # self.cam = Camera(device=0)
        self.cam = None

        # setting up arduino communication
        self.arduino_port = '/dev/ttyACM0'
        self.baud_rate = 9600

        #self.setup_arduino()

    def setup_arduino(self):
        with serial.Serial(self.arduino_port, self.baud_rate, timeout=1) as ser:
                # initialise arduino
                time.sleep(2)
                ser.flushInput()
                ser.flushOutput()

    def send_command(self, command):
        try:
            with serial.Serial(self.arduino_port, self.baud_rate, timeout=1) as ser:  # Ensure serial connection
                time.sleep(2)  # Give Arduino time to initialize
                ser.flushInput()  # Clear input buffer
                ser.flushOutput()  # Clear output buffer

                # Send command
                print(f"Sending command: {command}")
                ser.write(f"{command}\n".encode())
                time.sleep(0.5)  # Give Arduino time to process
        except serial.SerialException as e:
            print(f"Error in serial communication: {e}")
            return None

    
    def update_state(self, response):
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
        self.state[2] = (self.state[2] + ang) % (2 * np.pi)

        # update state
        if distance > 0:
            self.state[0] += distance * np.cos(self.state[2])
            self.state[1] += distance * np.sin(self.state[2])

    def __calc_revs__(self, dis):
        return dis / (np.pi * self.wheel)
    
    def __calc_dis__(self, point):
        return np.sqrt((point[0] - self.state[0])**2 + (point[1] - self.state[1])**2)
    
    def __calc_ang__(self, point):
        ang = np.arctan2(point[1] - self.state[1], point[0] - self.state[0])
        ang = (ang - self.state[2]) % (2 * np.pi)
        if ang > np.pi: ang -= 2*np.pi
        return ang
    
    def drive_to_box(self, point):
        dis = self.__calc_dis__(point)
        ang = self.__calc_ang__(point)

        while dis > 0.2:
            if abs(ang) > 0.005: self.rotate(ang)
            self.drive(1.0)

            # recalculating position
            dis = self.__calc_dis__(point)
            ang = self.__calc_ang__(point)
        
    def drive_to_target(self):
        # detection threshold
        cam_min = 0.08

        # detecting balls
        try:
            d, t = self.cam.detect_closest()
            # driving until 20cm away and ball is centered
            while d > cam_min:
                try:
                    if abs(t) > 0.005: self.rotate(t)
                    self.drive(0.05)
                    print(f"distance: {d}, angle: {t}")
                    
                    # re-detection
                    d, t = self.cam.detect_closest()
                except:
                    print('error: ball not found')
                    return None
            
            # drive 20cm blind due to camera limitations
            #self.drive(self.__calc_revs__(cam_min))
        except:
            print("no balls found")
        return True

    def drive(self, revs):
        response = self.send_command(f'$drive: {revs} rev')
        self.update_state(response)

    def rotate(self, ang):
        response = self.send_command(f'$turn: {ang} rad')
        self.update_state(response)

    def flip(self):
        self.send_command(f'$flip: theta 180 DT 60')
        
        
if __name__ == "__main__":
    bot = Bot()
    bot.rotate(np.pi)

