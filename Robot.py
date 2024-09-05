import math
import time
from gpiozero_extended import Motor, PID

class RobotController:
    def __init__(self, left_motor_pins, right_motor_pins, wheel_diameter, encoder_ppr, pid_params, wheel_base):

        #############################
        #Drive Initializing
        # Initialize Motor classes for both left and right motors
        self.left_motor = Motor(
            enable1=left_motor_pins.get('enable1'),
            enable2=left_motor_pins.get('enable2'),
            pwm1=left_motor_pins.get('pwm1'),
            pwm2=left_motor_pins.get('pwm2'),
            encoder1=left_motor_pins.get('encoder1'),
            encoder2=left_motor_pins.get('encoder2'),
            encoderppr=encoder_ppr
        )

        self.right_motor = Motor(
            enable1=right_motor_pins.get('enable1'),
            enable2=right_motor_pins.get('enable2'),
            pwm1=right_motor_pins.get('pwm1'),
            pwm2=right_motor_pins.get('pwm2'),
            encoder1=right_motor_pins.get('encoder1'),
            encoder2=right_motor_pins.get('encoder2'),
            encoderppr=encoder_ppr
        )

        # Initialize PID controllers for both motors
        self.left_pid = PID(
            Ts=pid_params['Ts'], kp=pid_params['kp'], ki=pid_params['ki'], kd=pid_params['kd']
        )
        self.right_pid = PID(
            Ts=pid_params['Ts'], kp=pid_params['kp'], ki=pid_params['ki'], kd=pid_params['kd']
        )

        self.wheel_diameter = wheel_diameter
        self.wheel_circumference = wheel_diameter * math.pi  # Pi * diameter
        self.encoder_ppr = encoder_ppr
        self.wheel_base = wheel_base
        self.pose = [0, 0,0]  # [x, y,theta] , -pi<=theta<=pi
        #############################
        #Camera Class Initializing

        #############################
        #Arm and release servo Initializing

        #############################
        #Arena Initializing
        self.arena = [4,6] #X and Y
        self.box_location = [4,0] #Location of Drop box
        #############################

    def calculate_revolutions(self, distance):
        return distance / self.wheel_circumference

    def drive(self, revolutions):
        """
        Drive the robot forward by a specified number of wheel revolutions using PID control.

        :param revolutions: Number of revolutions to drive forward.
        :type revolutions: float
        """
        # Reset the encoder angles to start from zero
        self.left_motor.reset_angle()
        self.right_motor.reset_angle()

        # Target encoder steps for the specified number of revolutions
        target_steps = self.encoder_ppr * revolutions

        while True:
            # Get the current steps for both motors
            left_steps = self.left_motor.get_angle() / 360 * self.encoder_ppr
            right_steps = self.right_motor.get_angle() / 360 * self.encoder_ppr

            # Calculate PID output
            left_output = self.left_pid.control(xsp=target_steps, x=left_steps)
            right_output = self.right_pid.control(xsp=target_steps, x=right_steps)

            # Set motor outputs
            self.left_motor.set_output(left_output)
            self.right_motor.set_output(right_output)

            # Break if both motors have completed the specified number of revolutions
            if left_steps >= target_steps and right_steps >= target_steps:
                break

            time.sleep(0.01)  # Small delay for loop

        # Stop both motors
        self.left_motor.set_output(0)
        self.right_motor.set_output(0)

        # Update the position
        self.pose[0] += self.wheel_circumference * revolutions  

    def turn(self, radians):
        """
        Turn the robot by a specified angle in radians using PID control.

        :param radians: Angle to turn in radians.
        :type radians: float
        """
        # Calculate the distance each wheel needs to travel for the turn
        turn_circumference = self.wheel_base * abs(radians)
        revolutions = self.calculate_revolutions(turn_circumference)

        # Reset the encoder angles to start from zero
        self.left_motor.reset_angle()
        self.right_motor.reset_angle()

        # Target encoder steps for the specified number of revolutions
        target_steps = self.encoder_ppr * revolutions

        while True:
            # Get the current steps for both motors
            left_steps = self.left_motor.get_angle() / 360 * self.encoder_ppr
            right_steps = self.right_motor.get_angle() / 360 * self.encoder_ppr

            # Calculate PID output
            if radians > 0:  # Turning right
                left_output = self.left_pid.control(xsp=target_steps, x=left_steps)
                right_output = -self.right_pid.control(xsp=target_steps, x=right_steps)
            else:  # Turning left
                left_output = -self.left_pid.control(xsp=target_steps, x=left_steps)
                right_output = self.right_pid.control(xsp=target_steps, x=right_steps)

            # Set motor outputs
            self.left_motor.set_output(left_output)
            self.right_motor.set_output(right_output)

            # Break if both motors have completed the specified number of revolutions
            if left_steps >= target_steps and right_steps >= target_steps:
                break

            time.sleep(0.01)  # Small delay for loop

        # Stop both motors
        self.left_motor.set_output(0)
        self.right_motor.set_output(0)

        # Update the heading
        self.pose[2] += radians

        
        # Normalize the heading
        self.pose[2] = (self.pose[2] + math.pi) % (2 * math.pi) - math.pi

    def get_position(self):
        return self.pose
    
    def return_to_start(self):
        pass

    #########################    
    #Tennis BALL handling FUNCTIONALITY
    
    
    def swing_arm(self):
        pass
    
    def collecting_ball(self):
        "Use of Steady driving and ultrasonic to activate the arm swing"
        "This method is called after run method is closer to the object approx 0.3m"
        pass

    def release_storage(self):
        "This method will be flagged to run after the robot is collected 3-4 balls or it didn't find anymore balls or time is up"
        pass

    def is_valid_ball(self, ball_location):
        """
        Check if the ball's estimated location is within the bounds of the arena.

        """
        x, y = ball_location
        arena_width, arena_length = self.arena

        return 0 <= x <= arena_width and 0 <= y <= arena_length

    #########################
    def run(self):
        
        pass


    #########################
    def cleanup(self):
        """Cleanup GPIO resources."""
        del self.right_motor, self.left_motor
    
if __name__ == "__main__":

    #Motor PINS
    left_motor_pins = {
        'enable1': 16,
        'enable2': 17,
        'pwm1': 18,
        'pwm2': 19,
        'encoder1': 24,
        'encoder2': 25
    }
    right_motor_pins = {
        'enable1': 20,
        'enable2': 21,
        'pwm1': 22,
        'pwm2': 23,
        'encoder1': 26,
        'encoder2': 27
    }

    #Robot Parameters
    wheel_diameter = 0.1  
    encoder_ppr = 450
    pid_params = {'Ts': 0.01, 'kp': 0.15, 'ki': 0.35, 'kd': 0.01}
    wheel_base = 0.3 
    robot = RobotController(left_motor_pins, right_motor_pins, wheel_diameter, encoder_ppr, pid_params, wheel_base)

    #Sevo Pins



    #Ultrasonic Pins


    


    # robot.drive(revolutions=2)
    # robot.turn(radians=math.pi / 2)  # Turn 90 degrees to the right
    # robot.turn(radians=-math.pi / 2)  # Turn 90 degrees to the left
    # print(robot.get_position())
