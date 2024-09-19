import time
import serial

# Serial communication with Arduino (adjust the port as needed)
arduino_port = '/dev/ttyACM0'  # Adjust this to your Arduino port
baud_rate = 9600

def send_command(command):
    """ Sends a command to the Arduino and waits for a response. """
    try:
        with serial.Serial(arduino_port, baud_rate, timeout=1) as ser:  # Ensure serial connection
            time.sleep(2)  # Give Arduino time to initialize
            ser.flushInput()  # Clear input buffer
            ser.flushOutput()  # Clear output buffer

            # Send command
            print(f"Sending command: {command}")
            ser.write(f"{command}\n".encode())
            time.sleep(0.5)  # Give Arduino time to process
            
            # Read response
            if ser.in_waiting > 0:  # Check if there is data to read
                response = ser.readline().decode().strip()
                print(f"Response received: {response}")
                return response
            else:
                print("No response from Arduino.")
                return None
    except serial.SerialException as e:
        print(f"Error in serial communication: {e}")
        return None

if __name__ == "__main__":
    while True:
        # Prompt user for input
        user_input = input("Enter 1 for Drive command, 2 for Flip command, or q to quit: ")

        if user_input == '1':
            # Call Drive command
            print(send_command("$Drive: 1 Rev"))
        elif user_input == '2':
            # Call Flip command
            print(send_command("$Flip Theta 180 DT 45"))
        elif user_input.lower() == 'q':
            # Exit the loop
            print("Exiting...")
            break
        else:
            print("Invalid input. Please enter 1, 2, or q to quit.")
