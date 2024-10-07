from gpiozero import DistanceSensor
import time
from time import sleep
from bot import Bot


# Define the trigger and echo pins
TRIG = 23  # Trigger Pin (GPIO 23)
ECHO = 24  # Echo Pin (GPIO 24)

# Create a DistanceSensor object
sensor = DistanceSensor(echo=ECHO, trigger=TRIG)

# Calibration factor to correct the distance
calibration_factor = 1  # Adjust as needed

def measure_distance():
    distance = sensor.distance * 100  # Get distance in cm
    return distance * calibration_factor  # Apply calibration factor

def check_distance(samples=10, threshold=100): #Change threshold value as needed
    distances_below_threshold = 0
    for _ in range(samples):
        dist = measure_distance()
        # Ignore distances greater than 100 cm
        if dist > 100:
            continue  # Skip this iteration        
        if dist < threshold:
            distances_below_threshold += 1
        
        sleep(1)  # Short delay between samples
    
    # If more than 6 out of 10 samples are below the threshold
    if distances_below_threshold > 6:
        return True
    else:
        return False

def drive_backward():
    # Bot().drive(0.5)
    print("Driving backward...")  # Replace with actual motor control code

def stop_robot():
    # Bot().drive(0)
    print("Stopping the robot...")  # Replace with actual motor stop code

def ultrasonic():
    try:
        while True:
            # Check if more than 6 out of 10 samples are less than threshold cm
            if check_distance():
                drive_backward()
                # Keep running backward until 10 consecutive samples are less than 7 cm
                consecutive_samples = 0
                while consecutive_samples < 10:
                    dist = measure_distance()
                    # Ignore distances greater than 100 cm
                    if dist > 100:
                        continue  # Skip this iteration
                    # Print distance only if it's less than 50 cm
                    if dist < 100:                          # CHANGE THIS TO A VALUE OF USERS CHOICE
                        print(f"Current Distance: {dist:.2f} cm")
                    if dist < 7:                          # CHANGE THIS TO A VALUE OF USERS CHOICE
                        consecutive_samples += 1
                    else:
                        consecutive_samples = 0  # Reset the count if a sample is greater than 7 cm
                    sleep(0.1)  # Check distance continuously with a small delay
                stop_robot()  # Stop the robot after 10 consecutive samples less than 7 cm
                break  # Exit the loop and finish the code run
            sleep(1)  # Delay before checking again

    except KeyboardInterrupt:
        print("Measurement stopped by User")

# Call the ultrasonic function to run the process
ultrasonic()