#include <Encoder.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

#define SERVO_PIN 10

Servo myServo;
Encoder myEnc(2, 3);
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  
  // Gradually approach position 170 from 150
  for (int i = 150; i <= 170; i++) {
    myServo.write(i);
    delay(50); // Adjust delay as needed for desired speed
  }
  delay(5000);
  
  AFMS.begin();
  myMotor->setSpeed(50); // Set the speed of the motor to a lower value for slight movements
  myMotor->run(BACKWARD); // Set the initial direction of the motor (opposite of servo)
}

long oldPosition = -999;
int frequency = 100; // Frequency for servo movement
int period = 2000;
unsigned long servoTimeStamp = 0; // Time stamp for servo movement
int command = 0;
int sp = 600;
bool servoMoving = true; // Flag to track servo movement
bool openingComplete = false; // Flag to indicate if the opening movement is complete
bool delayComplete = false; // Flag to indicate if the delay is complete

void loop() {
  // Servo movement
  if (servoMoving && millis() > servoTimeStamp + frequency) {
    servoTimeStamp = millis();
    if (myServo.read() > 60) {
      myServo.write(myServo.read() - 4); // Move servo in small decrements towards 60 degrees (fully open position)
    } else {
      // Stop the servo movement when fully extended
      servoMoving = false;
      delayComplete = false; // Reset delay flag
    }
  } else if (!servoMoving && myServo.read() < 170) {
    // If servo has stopped and not fully closed, gradually close the arm
    if (millis() > servoTimeStamp + frequency) {
      myServo.write(myServo.read() + 4);
      servoTimeStamp = millis();
    }
  }

  // Calculate command for the DC motor based on the command for the servo motor
  if (servoMoving || (!servoMoving && myServo.read() < 170)) {
    if (!openingComplete) {
      command = 170 - myServo.read(); // Mirror the movement of the servo in the opposite direction
    } else {
      command = myServo.read() - 170; // Reverse direction for closing movement
    }
    
    // Set direction of the DC motor based on the command
    if (command < 0) {
      myMotor->run(BACKWARD); // If command is negative, run the motor backward (opposite of servo)
    } else {
      myMotor->run(FORWARD); // If command is positive, run the motor forward (opposite of servo)
    }

    // Set speed of the DC motor based on the absolute value of the command
    myMotor->setSpeed(abs(command));
  } else {
    // Stop the DC motor when the servo stops, and during the delay
    if (!delayComplete) {
      myMotor->setSpeed(0);
      delayComplete = true; // Set delay flag after stopping the motor
      delay(1000); // Brief pause between opening and closing movements
    }
  }

  // Update openingComplete flag when opening movement is complete
  if (!servoMoving && !openingComplete) {
    openingComplete = true;
  }
}

int getEncoderCount() {
  return myEnc.read();
}

double findError(int setP) {
  return setP - getEncoderCount();
}

double kp = 1.0;
double getProportionalCommand(double err) {
  return kp * err;
}

double ki = 0.001;
double integral = 0;
double maxIntegral = 800;

double getIntegralControl(double err) {
  integral += err;
  if (abs(integral) > abs(maxIntegral)) {
    if (integral < 0) {
      //integral= -1*maxIntegral;
    } else {
      //integral=maxIntegral;
    }
  }
  return ki * err;
}

double getCommand(double setP) {
  double err = findError(setP);
  double p = getProportionalCommand(err);
  double i = getIntegralControl(err);

  return p + i;
}

