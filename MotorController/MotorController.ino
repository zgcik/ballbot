#include <Servo.h>

// pin definitions
const int rightEncoderPin = 2; // right motor encoder (interrupt pin)
const int leftEncoderPin = 3;  // left motor encoder (interrupt pin)

const int rightMotorEN = 11;   // right motor enable (PWM pin)
const int rightMotorIn1 = 10;   // right motor direction pin 1
const int rightMotorIn2 = 9;   // right motor direction pin 2

const int leftMotorEN = 6;     // left motor enable (PWM pin)
const int leftMotorIn1 = 8;    // left motor direction pin 1
const int leftMotorIn2 = 7;    // left motor direction pin 2

// Define pins for Ultrasonic Sensor
const int trigPin = 4;      // Trigger pin of the ultrasonic sensor
const int echoPin = 13;      // Echo pin of the ultrasonic sensor

// Variables for distance measurement and counting
int distance = 0;
int count = 0;

// PID class
class PID{
  private:
    float kp;
    float ki;
    float kd;

    float last_error;
    float integral;
    unsigned long last_time;

  public:
  // constructor:
    PID(float kp_in, float ki_in, float kd_in){
      kp = kp_in;
      ki = ki_in;
      kd = kd_in;

      last_error = 0.0;
      integral = 0.0;
      last_time = millis();
    }
  
    float compute(float setpoint, float val){
      unsigned long t = millis();
      float t_elapsed = (t - last_time) / 1000.0;
      
      if (t_elapsed <= 0) return 0;

      // calculate error
      float error = setpoint - val;

      // integral term
      integral += error * t_elapsed;

      // derivative term
      float derivative = (error - last_error) / t_elapsed;

      // update last values
      last_error = error;
      last_time = t;

      // return the control signal
      return (kp * error) + (ki * integral) + (kd * derivative);
    }
};

PID linPID(0.0, 0.01, 0.01);
PID rotPID(0.0, 0.00, 0.0);

int ticksPerRevolution = 900; // 900 encoder ticks per revolution
volatile long leftTicks = 0;
volatile long rightTicks = 0;

float baseline = 0.2;  // default baseline
float wheelRadius = 0.05415049396898059;  // default wheel radius

Servo myServo;

// interrupt service routine for left encoder
void leftEncoderISR() {
  leftTicks++;
}

// interrupt service routine for right encoder
void rightEncoderISR() {
  rightTicks++;
}


float normalizeAngle(float angle) {
  while (angle > PI) {
    angle -= 2 * PI;
  }
  while (angle < -PI) {
    angle += 2 * PI;
  }
  return angle;
}

// function to drive the left motor at constant speed
void driveLeftMotor(int pwmValue) {
  if (pwmValue > 0){
    digitalWrite(leftMotorIn1, HIGH);
    digitalWrite(leftMotorIn2, LOW);
    analogWrite(leftMotorEN, pwmValue);
  }
  if (pwmValue < 0){
    digitalWrite(leftMotorIn1, LOW);
    digitalWrite(leftMotorIn2, HIGH);
    analogWrite(leftMotorEN, abs(pwmValue));
  }
}


// function to adjust right motor speed based on PID
void driveRightMotor(int pwmValue) {
  if (pwmValue > 0){
    digitalWrite(rightMotorIn1, HIGH);
    digitalWrite(rightMotorIn2, LOW);
    analogWrite(rightMotorEN, pwmValue);
  }
  if (pwmValue < 0){
    digitalWrite(rightMotorIn1, LOW);
    digitalWrite(rightMotorIn2, HIGH);
    analogWrite(rightMotorEN, abs(pwmValue));
  }

}


// function to stop both motors
void stopMotors() {
  digitalWrite(leftMotorIn1, LOW);
  digitalWrite(leftMotorIn2, LOW);
  analogWrite(leftMotorEN, 0);

  digitalWrite(rightMotorIn1, LOW);
  digitalWrite(rightMotorIn2, LOW);
  analogWrite(rightMotorEN, 0);
}

// Function to measure distance using the ultrasonic sensor
int getDistance() {
  // Send a pulse to the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(echoPin, HIGH);

  // Convert the duration to distance in cm
  int distance = duration * 0.034 / 2;
  
  return distance;
}


void drive(float revolutions) {
  long targetTicks = revolutions * ticksPerRevolution;
  leftTicks = 0;
  rightTicks = 0;
  int pwm = 175; // 50%

  while (abs(leftTicks) < abs(targetTicks) && abs(rightTicks) < abs(targetTicks)) {
    if (revolutions > 0){
      // call the PID function to adjust the right motor
      float control = linPID.compute(leftTicks, rightTicks);

      // constrain control signal to valid PWM range
      control = constrain(fabs(control), pwm-50, 255);

      // drive motors
      driveLeftMotor(pwm);  // drive left motor at constant speed
      driveRightMotor(control);  // adjust right motor speed based on PID control
    }
    else if (revolutions < 0){
       // call the PID function to adjust the right motor
      float control = linPID.compute(leftTicks, rightTicks);

      // constrain control signal to valid PWM range
      control = constrain(fabs(control), pwm-5, 255);

      // drive motors
      driveLeftMotor(-pwm);  // drive left motor at constant speed
      driveRightMotor(-control);  // adjust right motor speed based on PID control

    }
  }

  // output encoder ticks to the serial monitor
  Serial.print("L:");
  Serial.print((float)leftTicks/(float)ticksPerRevolution);
  Serial.print(",R:");
  Serial.println((float)rightTicks/(float)ticksPerRevolution);

  // stop motors when done
  stopMotors();
}


// function to move the servo incrementally by 10 degrees
void moveServo(int dt, int targetAngle) {
  int currentAngle = myServo.read();  // get the current angle of the servo
  
  if (targetAngle > currentAngle) {
    // incrementally move the servo up to the target angle
    for (int angle = currentAngle; angle <= targetAngle; angle += 10) {
      myServo.write(angle);
      delay(dt);  // wait for the given time step
    }
  } else {
    // decrementally move the servo down to the target angle
    for (int angle = currentAngle; angle >= targetAngle; angle -= 10) {
      myServo.write(angle);
      delay(dt);  // wait for the given time step
    }
  }
}


void setup() {
  // set up motor control pins as outputs
  pinMode(rightMotorIn1, OUTPUT);
  pinMode(rightMotorIn2, OUTPUT);
  pinMode(rightMotorEN, OUTPUT);

  pinMode(leftMotorIn1, OUTPUT);
  pinMode(leftMotorIn2, OUTPUT);
  pinMode(leftMotorEN, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // set motors to stopped state at the start
  stopMotors();
  // set up encoder pins as inputs
  pinMode(rightEncoderPin, INPUT);
  pinMode(leftEncoderPin, INPUT);

  //Servo 
  myServo.attach(5);  // attach the servo to pin 11
  myServo.write(7);    // initialize the servo at 0 degrees

  // attach interrupts for encoder reading
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);

  Serial.begin(9600); // initialize serial communication at 9600 baud
}


// turn function
void turn(float radians) {
  // normalize the angle to [-pi, pi]
  radians = normalizeAngle(radians);
  
  // calculate the number of wheel rotations required to turn the robot by the specified angle
  float distanceToTravel = (fabs(radians) * baseline);  // arc length for one wheel
  float wheelRotations = distanceToTravel / (2 * PI * wheelRadius);  // required wheel rotations
  long targetTicks = wheelRotations * ticksPerRevolution;  // convert to encoder ticks

  // reset the encoder ticks
  leftTicks = 0;
  rightTicks = 0;

  int pwm = 150;

  // perform the turn
  if (radians > 0) {
    // turning right
    while (abs(leftTicks) < targetTicks && abs(rightTicks) < targetTicks) {
      float control = rotPID.compute(abs(leftTicks), abs(rightTicks));
      control = constrain(fabs(control), 0, 255);

      driveRightMotor(-control);   // move right motor
      driveLeftMotor(pwm);  // move left motor
      
    }
  } else if (radians < 0) {
    // turning left
    while (abs(leftTicks) < targetTicks && abs(rightTicks) < targetTicks) {
      float control = rotPID.compute(abs(leftTicks), abs(rightTicks));
      control = constrain(fabs(control), 0, 255);

      driveLeftMotor(-pwm);  // move left motor
      driveRightMotor(control);     // move right motor
    }
  }

  // output encoder ticks to the serial monitor
  Serial.print("L:");
  Serial.print((float)leftTicks/(float)ticksPerRevolution);
  Serial.print(",R:");
  Serial.println((float)rightTicks/(float)ticksPerRevolution);

  // stop both motors after the turn is completed
  stopMotors();
}

void drivelr(float left, float right){

}

void loop() {
  delay(1000);
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // read input from serial

    // check if the input is for PID tuning
    // Check if the input starts with "$Drive:"
    if (input.startsWith("$drive:")) {
      String revStr = input.substring(7);  // extract substring after "$Drive:"
      int indexOfRev = revStr.indexOf("rev");
      if (indexOfRev != -1) {
        revStr = revStr.substring(0, indexOfRev);  // extract the number part
        revStr.trim();  // trim any whitespace
        float rev = revStr.toFloat();
        if (rev > 0) {
          drive(rev);  // call the drive function with the extracted number of revolutions
        } else {
          //Serial.println("error: invalid number of revolutions.");
          drive(rev);
        }
      } else {
        Serial.println("error: invalid command format.");
      }
    }
    // check if the input starts with "$Turn:"
    else if (input.startsWith("$turn:")) {
      String radStr = input.substring(6);  // extract substring after "$Turn:"
      int indexOfRad = radStr.indexOf("rad");
      if (indexOfRad != -1) {
        radStr = radStr.substring(0, indexOfRad);  // extract the radians part
        radStr.trim();  // trim any whitespace
        float radians = radStr.toFloat();  // can handle negative values
        Serial.println(radians);
        turn(radians);  // call the turn function with the extracted angle in radians
      } else {
        Serial.println("error: invalid command format.");
      }
    }
    // check if the input starts with "$SetBaseline:"
    else if (input.startsWith("$setBaseline:")) {
      int commandLength = 13;  // length of "$setBaseline:"
      String baselineStr = input.substring(commandLength);  // extract the number after "$setBaseline:"
      baselineStr.trim();  // trim any whitespace
      float newBaseline = baselineStr.toFloat();  // convert to float
      if (newBaseline > 0) {
        baseline = newBaseline;  // update the baseline
      } else {
        Serial.println("error: invalid baseline value.");
      }
    }
    // check if the input starts with "$SetWheelRadius:"
    else if (input.startsWith("$setWheelRadius:")) {
      int colonIndex = input.indexOf(':');
      if (colonIndex != -1 && colonIndex + 1 < input.length()) {
        String wheelRadiusStr = input.substring(colonIndex + 1);  
        wheelRadiusStr.trim();  // trim any whitespace
        float newWheelRadius = wheelRadiusStr.toFloat();  
        if (newWheelRadius > 0) {
          wheelRadius = newWheelRadius;  // update the wheel radius
        } else {
          Serial.println("error: invalid wheel radius value.");
        }
      } else {
        Serial.println("error: invalid command format.");
      }
    }
    // ceck if the input starts with "$Flip Theta"
    else if (input.startsWith("$flip:")) {
      int indexOfTheta = input.indexOf("theta");
      int indexOfDT = input.indexOf("DT");
      if (indexOfTheta != -1 && indexOfDT != -1) {
        String thetaStr = input.substring(indexOfTheta + 6, indexOfDT - 1);  // extract the angle part
        String dtStr = input.substring(indexOfDT + 3);  // extract the time step part
        thetaStr.trim();
        dtStr.trim();
        int targetAngle = thetaStr.toInt();  // convert to integer
        int dt = dtStr.toInt();  // convert to integer
        if (targetAngle > 0 && dt > 0) {
          moveServo(dt, targetAngle);  // move the servo to the target angle
          delay(1000);  // wait for 1 second
          moveServo(dt, 0);  // move the servo back to 0 degrees
          moveServo(dt, 7);
        } else {
          Serial.println("error: invalid theta or DT values.");
        }
      } else {
        Serial.println("error: invalid flip command format.");
      }
    } 
    
    else if(input.startsWith("$Ultrasonic:")){
      // Get the distance from the ultrasonic sensor
      distance = getDistance();
      
      // Print the distance to the Serial Monitor
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      while (distance >=5){
        // Check if the distance is less than 30 cm
        if (distance < 30) {
          count++;  // Increment count if the distance is less than 30 cm
        } else {
          count = 0;  // Reset count if the distance is more than 30 cm
        }

        // If the count exceeds 5, move the robot backward
        if (count >= 5) {
          drive(-1);
          count = 0;  // Reset count after moving backward
        }

        // Delay to allow some time between measurements
        delay(1000);
      }
    } else {
      Serial.println("error: unrecognized command.");
    }
  }
}


// function to drive the robot a certain number of revolutions




