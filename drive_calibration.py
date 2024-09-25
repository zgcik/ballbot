import numpy as np
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
    return scale

    


def baseline_calibration(bot):
    test_vals = [np.pi, 2*np.pi, 3*np.pi, 4*np.pi]
    revs_out = []

    for val in test_vals:
        print(f'turning {val} rads')
        
        while True:
            revs_in = input('input the number of revs to turn')
            try:
                revs_in = float(revs_in)
            except ValueError:
                print('revs must be a number')
                continue

            bot.send_command(f'$drivelr: {revs_in, -revs_in} rev')

            user_in = input(f'did the robot turn {val} rads ? [y/N]').strip().lower()
            if user_in == 'y':
                revs_out.append(revs_in)
                print(f'recording that the robot turned {val} rads in {revs_in} revs')
                break
    
    # calculate the average wheelbase using the revolutions and angle
    total_baseline = 0
    for i in range(len(test_vals)):
        if revs_out[i] > 0:
            wheel_diameter = bot.wheel
            baseline = (test_vals[i] * wheel_diameter) / revs_out[i]
            total_baseline += baseline

    # calculate the average baseline
    if len(revs_out) > 0:
        ave_baseline = total_baseline / len(revs_out)
    else:
        ave_baseline = None

    print(f'average baseline: {ave_baseline} m')
    return ave_baseline


if __name__ == "__main__":
    bot = Bot()