import RPi.GPIO as GPIO
import numpy as np

from camera import Camera
from motor import Motor

class Bot:
    def __init__(self):
        self.pose = [0, 0, 0]

        # pin assignments
        self.pins = {
            'motora': {
                'in1': 17,
                'in2': 27,
                'ena': 18,
                'enc': 5
            },
            
            'motorb': {
                'in3': 22,
                'in4': 23,
                'ena': 19,
                'enc': 12
            }
        }
        
        # initalizing pins
        self.setupGPIO()

        # init cam
        self.cam = Camera(device=2)

        # init motors
        self.pwm_freq = 10000

        # motor a: right
        self.motora = Motor(
            pwm_freq= self.pwm_freq,
            in1= self.pins['motora']['in1'],
            in2= self.pins['motora']['in2'],
            ena= self.pins['motora']['ena'],
            enc= self.pins['motora']['enc']
        )

        # motor b: left
        self.motorb = Motor(
            pwm_freq= self.pwm_freq,
            in1= self.pins['motorb']['in3'],
            in2= self.pins['motorb']['in4'],
            ena= self.pins['motorb']['ena'],
            enc= self.pins['motorb']['enc']
        )
    
    def setupGPIO(self):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.pins['motora']['in1'], GPIO.OUT)
        GPIO.setup(self.pins['motora']['in2'], GPIO.OUT)
        GPIO.setup(self.pins['motora']['ena'], GPIO.OUT)

        GPIO.setup(self.pins['motorb']['in3'], GPIO.OUT)
        GPIO.setup(self.pins['motorb']['in4'], GPIO.OUT)
        GPIO.setup(self.pins['motorb']['ena'], GPIO.OUT)

        GPIO.setup(self.pins['motora']['enc'], GPIO.IN)
        GPIO.setup(self.pins['motorb']['enc'], GPIO.IN)

        GPIO.add_event_detect(self.pins['motora']['enc'], GPIO.RISING, callback=self.motora.enc_cb())
        GPIO.add_event_detect(self.pins['motorb']['enc'], GPIO.RISING, callback=self.motorb.enc_cb())

    def update_pose(self, dis, ang):
        if dis > 0:
            self.pose[0] += dis * np.cos(self.pose[2])
            self.pose[1] += dis * np.sin(self.pose[2])
        
        self.pose[2] = (self.pose[2] + ang) % (2 * np.pi)

    def drive(self, dis):
        pass

    def turn(self, ang):
        pass