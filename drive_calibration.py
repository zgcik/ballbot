import numpy as np
import os
from bot import Bot

def wheel_calibration(bot):
    test_vals = [0.5, 1.0, 1.5, 2.0]
    revs_out = []
    
    for val in test_vals:
        print(f'driving {val} m')

        while True:
            revs_in = input('input the number of revs to drive:')
            try:
                revs_in = float(revs_in)
            except ValueError:
                print('revs must be a number')
                continue

            bot.drive(revs_in)

            user_in = input(f'did the robot travel {val} m? [y/N]').strip().lower()
            if user_in == 'y':
                revs_out.append(revs_in)
                print(f'recording that the robot drove {val} m in {revs_in} revs')
                break
    
    # compute the scale parameter through averaging
    total_diameter = 0
    for i in range(len(test_vals)):
        if revs_out[i] > 0:  # Ensure not to divide by zero
            diameter = (test_vals[i] / revs_out[i]) / np.pi  # diameter in meters
            total_diameter += diameter

    # Calculate the average diameter
    if len(revs_out) > 0:
        scale = total_diameter / len(revs_out)
    else:
        scale = None

    print(f'average scale: {scale} m')

    script_dir = os.path.dirname(__file__)
    np.save(os.path.join(script_dir, "calibration", "wheel.npy"), scale)


def baseline_calibration(bot):
    test_vals = [np.pi/2, np.pi, 2*np.pi, 3*np.pi, 4*np.pi]  # Target angles to turn
    ang_out = []  # List to store the input angles that successfully turned the robot the target angle

    for val in test_vals:
        print(f'target: turn the robot {np.degrees(val)} deg')

        while True:
            u_input = input('input the angle to turn the robot (in deg): ')
            try:
                u_input = np.radians(float(u_input))
            except ValueError:
                print('input must be a number')
                continue

            bot.rotate(u_input)

            user_in = input(f'did the robot turn {np.degrees(val)} radians? [y/N] ').strip().lower()
            if user_in == 'y':
                ang_out.append(u_input)
                print(f'recording that the robot turned {val} radians with input angle {u_input}')
                break 
    
    total_baseline = 0
    for i in range(len(test_vals)):
        wheel_diameter = bot.wheel
        target_angle = test_vals[i]
        input_angle = ang_out[i]

        baseline = (target_angle * wheel_diameter) / input_angle
        total_baseline += baseline

    # Calculate the average baseline
    if len(ang_out) > 0:
        ave_baseline = total_baseline / len(ang_out)
    else:
        ave_baseline = None

    print(f'average baseline: {ave_baseline} m')

    script_dir = os.path.dirname(__file__)
    np.save(os.path.join(script_dir, "calibration", "baseline.npy"), ave_baseline)


if __name__ == "__main__":
    bot = Bot()
    # wheel_calibration(bot)
    baseline_calibration(bot)