import time
import serial
import numpy as np

from camera import Camera
import math
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
        self.arduino_port = '/dev/ttyACM1'
        self.baud_rate = 9600

        # Open the serial connection during initialization
        self.ser = None
        self.setup_arduino()

        self.d = 1000
        self.t = 1000

        self.wheelrad = 0.024
        self.baseline = 0.2666

    def setup_arduino(self):
        # Try connecting to /dev/ttyACM1 first, if it fails, try /dev/ttyACM0
        possible_ports = ['/dev/ttyACM1', '/dev/ttyACM0']

        for port in possible_ports:
            try:
                print(f"Trying to connect to {port}...")
                self.ser = serial.Serial(port, self.baud_rate, timeout=1)
                time.sleep(2)  # Give Arduino time to initialize
                self.ser.flushInput()
                self.ser.flushOutput()
                print(f"Successfully connected to {port}.")
                return  # Exit the method if successful
            except serial.SerialException as e:
                print(f"Error in serial communication setup on {port}: {e}")

        # If no port works, set self.ser to None
        print("Failed to connect to any serial port.")
        self.ser = None

    def restart_serial(self):
        print("Restarting the serial connection...")
        self.setup_arduino()

    def send_command(self, command):
        if self.ser is None or not self.ser.is_open:
            print("Serial connection not established or lost.")
            self.restart_serial()  # Restart the serial connection
            return None

        while True:
            try:
                # Send command
                print(f"Sending command: {command}")
                self.ser.write(f"{command}\n".encode())
                time.sleep(0.5)  # Give Arduino time to process
                # Call received to read the response
                return self.received()

            except (serial.SerialException, OSError) as e:
                print(f"Error in serial communication: {e}")
                time.sleep(0.5)
                self.restart_serial()  # Restart the serial connection on error
                return None

    def received(self):
        """Listens to the serial and checks for '$done' message with a timeout of 2 seconds."""
        if self.ser is None:
            return None

        start_time = time.time()  # Record the current time to track timeout
        timeout_duration = 15 # Timeout after 2 seconds

        while True:
            if self.ser.in_waiting > 0:  # Check if data is available to read
                response = self.ser.readline().decode().strip()
                print(f"Received: {response}")  # Print the received response for debugging
                if "$done" in response:  # Check if $done is part of the response
                    print("Command complete.")
                    return response
                elif "Left" in response:
                    self.update_state(response) 

            # Check if the loop has exceeded the timeout duration
            if time.time() - start_time > timeout_duration:
                print("Timeout: No response from Arduino after 2 seconds.")
                return "Timeout"

            time.sleep(0.1)  # Small delay to prevent CPU overload
    
    def update_state(self, response):
        parts = response.split()
        left_ticks = int(parts[2])
        right_ticks = int(parts[5])
        d_L = (left_ticks / 900) * 2 * math.pi * self.wheelrad # Left wheel distance
        d_R = (right_ticks / 900) * 2 * math.pi * self.wheelrad  # Right wheel distance

        # Step 2: Calculate change in orientation (Δθ)
        delta_theta = (d_L - d_R ) / self.baseline

        # Step 3: Calculate average distance traveled
        avg_distance = (d_L + d_R) / 2

        # Step 4: Calculate change in position (Δx, Δy)
        if delta_theta == 0:  # Robot is moving straight
            delta_x = avg_distance * math.cos(self.state[2])
            delta_y = avg_distance * math.sin(self.state[2])
            self.state[2] += delta_theta
            self.state[2] = (self.state[2]  + math.pi) % (2 * math.pi) - math.pi
        else:  # Robot is turning
            delta_x = avg_distance * math.cos(self.state[2] + delta_theta / 2)
            delta_y = avg_distance * math.sin(self.state[2] + delta_theta / 2)
            self.state[2] += delta_theta
            self.state[2] = (self.state[2]  + math.pi) % (2 * math.pi) - math.pi

        # Step 5: Update the robot's position and orientation
        self.state[0] +=delta_x
        self.state[1] += delta_y
        print(f"New State X: {self.state[0]}, Y: {self.state[1]}, Theta: {self.state[2]}")


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
                time.sleep(0.2)
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
            if abs(t) > 0.05: 
                self.rotate(t/2)
            print(d <= 0.2 and abs(t) < 0.2)
            if d >= 1:
                self.drive(2)
            elif d >= 0.5 and d < 1:
                self.drive(0.5)
            elif d > 0.35:
                self.drive(0.25)
            elif d > 0.2 and d < 0.35:
                self.drive(0.35)
                return True
            elif d <= 0.2 and abs(t) < 0.2:
                return True
            # time.sleep(1.5) 

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

    def drive_to_location(self, location):
        #Location is x,y
        while True:

            distance = self.__calc_dis__(location)
            angle = self.__calc_ang__(location)
            print(f"distance left: {distance} and location: {location}")
            # if angle > 0.08:
            if abs(angle) > 0.05:  # Rotate only if angle difference is significant
                self.rotate(angle)

            if distance >= 0.5:
                self.drive(1)
            elif distance >= 0.3 and distance < 0.5:
                self.drive(0.25)
            else:
                self.drive(0.2)
            if distance < 0.05:
                break
        

        
        
        

        
        
if __name__ == "__main__":
    try:
        bot = Bot()
        while True:
            # Check if 'q' is pressed
            if keyboard.is_pressed('q'):
                bot.flip(180, 60)  # Run bot.flip when 'q' is pressed
        # bot.rotate(1.5708)
        # location = [1,0]
        # bot.drive_to_location(location)
        # bot.drive(-4)
        # bot.collect()

        # ang = 0
        # while True:
            
        #     bot.drive_to_target()
        #     bot.rotate(0.08)
        #     ang += 0.08
                

        #     if ang > 6.28319:
        #         break

    except KeyboardInterrupt:
        print("\nScript interrupted by Ctrl+C")
    finally:
        # Ensure the camera is properly closed when interrupted
        bot.cam.cam.stop()
        print("Camera stopped and cleaned up.")
        
            

    
    
    



    

