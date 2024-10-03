import time
import serial
import numpy as np

from camera import Camera

class Bot:
    def __init__(self):
        # Calibration
        self.wheel = 0.05415049396898059
        self.baseline = 0.023

        # Init robot state
        self.state = [0.0, 0.0, 0.0]

        # Init cam
        self.cam = Camera(device=0)

        # Setting up Arduino communication
        self.arduino_port = '/dev/ttyACM0'
        self.baud_rate = 9600

        # Open the serial connection during initialization
        self.ser = None
        self.setup_arduino()

        self.d = 1000
        self.t = 1000

    def setup_arduino(self):
        try:
            self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Give Arduino time to initialize
            self.ser.flushInput()
            self.ser.flushOutput()
        except serial.SerialException as e:
            print(f"Error in serial communication setup: {e}")
            self.ser = None

    def send_command(self, command):
        if self.ser is None:
            print("Serial connection not established.")
            return None
        while True:
            try:
                # Send command
                print(f"Sending command: {command}")
                self.ser.write(f"{command}\n".encode())
                time.sleep(0.5)  # Give Arduino time to process

                # Call received to read the response
                return self.received()

            except serial.SerialException as e:
                print(f"Error in serial communication: {e}")
                time.sleep(0.5)
                self.setup_arduino()
                return None

    def received(self):
        """Listens to the serial and checks for '$done' message with a timeout of 2 seconds."""
        if self.ser is None:
            return None

        start_time = time.time()  # Record the current time to track timeout
        timeout_duration = 2  # Timeout after 2 seconds

        while True:
            if self.ser.in_waiting > 0:  # Check if data is available to read
                response = self.ser.readline().decode().strip()
                print(f"Received: {response}")  # Print the received response for debugging
                if "$done" in response:  # Check if $done is part of the response
                    print("Command complete.")
                    return response

            # Check if the loop has exceeded the timeout duration
            if time.time() - start_time > timeout_duration:
                print("Timeout: No response from Arduino after 2 seconds.")
                return "Timeout"

            time.sleep(0.1)  # Small delay to prevent CPU overload
    
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

    def __revs_to_dis__(self, revs):
        return revs * (np.pi * self.wheel)
    
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
        self.d = 1000
        self.t = 1000
        while True: #True
            try:
                time.sleep(2)
                d, t = self.cam.detect_closest()
                self.d = d
                self.t = t
            except:
                print('error: ball not found')
                if self.d <= 0.2 and abs(self.t) < 0.2:
                    return True
                else:
                    return False
            print(f"distance: {d}, angle: {t}")
            if abs(t) > 0.15: 
                self.rotate(t/4)
            print(d <= 0.2 and abs(t) < 0.2)
            if d >= 1:
                self.drive(1)
            elif d >= 0.5 and d < 1:
                self.drive(0.5)
            elif d > 0.35:
                self.drive(0.25)
            elif d > 0.2 and d < 0.35:
                self.drive(0.35)
                return True
            elif d <= 0.2 and abs(t) < 0.2:
                return True
            time.sleep(1.5) 

    def drive(self, revs):
        self.send_command(f'$drive: {revs} rev')
        self.state[0] += self.__revs_to_dis__(revs) * np.cos(self.state[2])  # x position
        self.state[1] += self.__revs_to_dis__(revs) * np.sin(self.state[2])  # y position

        
    def rotate(self, ang):
        self.send_command(f'$turn: {ang} rad')
        self.state[2] = (self.state[2] + ang) % (2 * np.pi)


    def flip(self,ang,dt):
        self.send_command(f'$flip: theta {ang} dt {dt}')


    def collect(self):
        self.send_command(f'$collect: 650 dt')


    def storage(self):
        self.send_command(f'$storage: 3500 dt')

        
        
if __name__ == "__main__":
    bot = Bot()
    # bot.rotate(1)
    
    #bot.drive(1)
    # bot.collect()

    ang = 0
    while True:
        
        bot.drive_to_target()
        bot.rotate(0.08)
        ang += 0.08
            

        if ang > 6.28319:
            break
        
        
            

    
    
    



    

