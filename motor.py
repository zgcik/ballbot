import RPi.GPIO as GPIO
import time
from pid import PID

class Motor:
    def __init__(self, pwm_freq, in1, in2, ena, enc):
        # pwm frequency
        self.freq = pwm_freq # 10000

        # pin assignments
        self.in1 = in1
        self.in2 = in2
        self.ena = ena
        self.enc = enc

        # pwm objects for motor speed control
        self.pwm = GPIO.PWM(self.ena, self.pwm_freq)
        self.pwm.start(0) # start with motors stopped

        # initializing encoder count to 0
        self.enc_count = 0

        # initalizing pid
        self.pid = PID(kp=0.01, ki=0.35, kd=0.01)

    def enc_cb(self):
        self.enc_count += 1

    def calc_enc_speed(self, enc_count, t_int, ppr=48 * 75):
        # TODO: CHECK
        revs = enc_count / ppr
        speed = (revs / t_int) * 60
        return speed
    
    def get_enc_speed(self, t_int=0.001):
        count = self.enc_count

        # wait for a specified time interval
        time.sleep(t_int)

        # calculate number of pulses in the interval
        pulses = self.enc_count - count

        # calculate speeds in rpm
        speed = self.calc_speed(pulses, t_int)
        return speed

    def set_motor_dir(self, dir=1):
        if dir:
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
    
    def set_motor_speed(self, pwm):
        self.pwm.ChangeDutyCycle(pwm)

    def control_speed(self):
        # calculating pid speed
        speed = self.get_enc_speed(t_int=0.001)
        pid_out = self.pid.compute(self.speed, speed)

        # updating motor speed
        self.set_motor_speed(pid_out)

    def stop(self):
        self.pwm.ChangeDutyCycle(0)
    
    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()