/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>
#include <Servo.h>
#define SERVO_PIN 10

Servo myServo;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  myServo.attach(SERVO_PIN);
  myServo.write(170);
  delay(2000);
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
}

void loop() {
  uint8_t i;

  Serial.print("tick");
  myServo.write(170);
  myMotor->run(FORWARD);
    myMotor->setSpeed(255);
    delay(1000);
    myServo.write(160);
    delay(1000);
    myServo.write(150);
    delay(1000);
    myServo.write(140);
    delay(1000);
    myServo.write(130);
    delay(1000);
    myServo.write(120);
    delay(1000);
    myMotor->setSpeed(0);

 

  Serial.print("tock");
  myServo.write(110);
  myMotor->run(BACKWARD);
   myMotor->setSpeed(255);
    delay(1000);
    myServo.write(120);
    delay(1000);
    myServo.write(130);
    delay(1000);
    myServo.write(140);
    delay(1000);
    myServo.write(150);
    delay(1000);
    myServo.write(160);
    myMotor->setSpeed(0);
    delay(1000);

  Serial.print("tech");
  myMotor->run(RELEASE);
  delay(1000);
}