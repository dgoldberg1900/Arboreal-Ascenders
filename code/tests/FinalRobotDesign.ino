#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>          

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600 default is 2500
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
//uint8_t servonum = 5;
void setup() {
  Serial.begin(9600);
  //Serial.println("8 channel Servo test!");

  pinMode(6,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT); 

  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  delay(3000);
}
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); //Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); //Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  //Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

// 6 5 4 is original
int servoNumbers[] = {6,5,4}; //Where 4 and 12 are shoulder joints
int angleToMicroPulse(int angle){
  return int(map(angle, 0,90, USMIN,USMAX));
}
int angleToPulse(int angle){
  return int(map(angle, 0,90, SERVOMIN,SERVOMAX));

}

void setServoPositions(int arr[]){

  for(int i = 0; i < 3; i++){
      int pulse = angleToPulse(arr[i]);
      pwm.setPWM(servoNumbers[i],0,pulse);
      updateStatus(i,arr[i]);
  }
}


int servoAngles[] = {40,50,50};

//int servoAngles[] = {25,40,65};
void updateStatus(int index, int angle){
  servoAngles[index] = angle;
}

int lowLimits[] = {35,0,0};
int highLimits[] = {90,70,60};
bool moveServo(int index, int angle){
  int nextAngle = servoAngles[index] + angle;
  if(nextAngle > highLimits[index] || nextAngle < lowLimits[index]){
    //Serial.println("Joint Locked " + index );
    return false;
  }else{
    //Serial.println("Working");
    updateStatus(index,nextAngle);
    return true;
  }
}

bool commandAllServos(){
  //Serial.println("Commanding Now");
  for( int i = 0; i < 3; i++){
      //Serial.println(servoAngles[i]);
      int pulse = angleToPulse(servoAngles[i]);
      pwm.setPWM(servoNumbers[i], 0, pulse);
  }
  return true;
  
}
bool moveArm(int angle){
  // Where angle is the center joint
  moveServo(0,-1*angle/2);
  moveServo(1,angle);
  moveServo(2,angle/2);
}

bool checkIfLimitsReached(bool checkHigh){
  int count = 0;
  for(int i = 0; i < 3; i++){
    if(checkHigh && servoAngles[i] == highLimits[i]){
      count++;
    }else if (!checkHigh && servoAngles[i] == lowLimits[i]){
      count++;
    }
  }
  if(count >= 3){
    return true;
  }else{
    return false;
  }
  
}


// This test is to drive shoulder and elbow joints
int lastStamp = 0;
int period = 500;
int pausePeriod = 2000;
//int startingPositions[] = {45,0,45};//angles
int phase = -1;

int counter = 0;
int counter2 = 0;
int counter3 = 0;


void loop() {
  if(phase == -1){
    commandAllServos();
    //analogWrite(10,250);//E 2
    //digitalWrite(11,HIGH);//in3
    //digitalWrite(12,LOW); // in4
    //delay(5000);
    //analogWrite(10,0);
    //delay(15000);
    //analogWrite(10,250);//E 2
    //digitalWrite(11,LOW);//in3
    //digitalWrite(12,HIGH); // in4
    //delay(2400);
    //analogWrite(10,0);
    delay(5000);
    phase++;
  }
  else if(phase == 0 ){
    /*
    int index = 0;  
    if(true && !moveServo(1,2)){
        Serial.println("MotorLimitReached");
    }else{
      Serial.println("Motor Moving!");
    }
    index = 1;
  
  */
    moveArm(2);
    delay(300);
    counter2++;
    lastStamp = millis();
    //check if extension is over, and loop to next phase
    if(counter2 == 12){
        //Serial.println("PHASE COMPLETED" + phase);
        phase++; 
    }
     
  }else if (phase == 1){
      delay(5000);
      phase++;
    }
  
  else if (phase == 2){
    //This is the compression phase
    moveArm(-2);
    counter2++;
    lastStamp = millis();
    delay(300);
    if(counter2 == 24){
        //Serial.println("Phase Completed" + phase);
        delay(5000);
        phase++; 
    }
  }
  else if (phase == 3){
    if (counter3 == 0){
      analogWrite(6,220);//E 1
      digitalWrite(8,HIGH);//in1
      digitalWrite(9,LOW); // in2
      delay(1600);
      analogWrite(6,75);
      counter3++;
    }
    moveArm(2);
    delay(300);
    counter2++;
    if (counter2 == 36){
      delay(500);
      phase++;
    }
  }
  else if (phase == 4){
    analogWrite(6,0);
    delay(1000);
    analogWrite(6,75);//E 2
    digitalWrite(8,LOW);//in3
    digitalWrite(9,HIGH); // in4
    delay(1000);
    analogWrite(10,250);//E 2
    digitalWrite(11,HIGH);//in3
    digitalWrite(12,LOW); // in4
    delay(2500);
    analogWrite(10,0); //E 2
    delay(1000);  
    phase++;
  }
  else if (phase == 5){
    moveArm(-2);
    counter2++;
    lastStamp = millis();
    delay(300);
    if(counter2 == 47){
    analogWrite(10,250);//E 2
    digitalWrite(11,LOW);//in3
    digitalWrite(12,HIGH); // in4
    delay(2400);
    analogWrite(10,0); //E 2
    delay(2000);
    }
    if(counter2 == 48){
      counter = 0;
      counter2 = 0;
      counter3 = 0;
      phase = 0;
    }
  }

  //Frequency of the whole loop determined by delay, may need to replace with empty loop. 
  delay(100);
  commandAllServos();

  // TOP CLAW
  if(phase == 0 && counter == 0){
  analogWrite(10,250);//E 2
  digitalWrite(11,HIGH);//in3
  digitalWrite(12,LOW); // in4
  delay(2500);
  analogWrite(10,0); //E 2
  /*delay(1000);
  analogWrite(6,180);//E 2
  digitalWrite(8,HIGH);//in3
  digitalWrite(9,LOW); // in4
  delay(1500);
  analogWrite(6,50); //E 2   */
  counter ++;
}
if(phase == 1){
  analogWrite(10,250);//E 2
  digitalWrite(11,LOW);//in3
  digitalWrite(12,HIGH); // in4
  delay(2400);
  analogWrite(10,0);
  delay(1000);
  analogWrite(6,220);//E 2
  digitalWrite(8,HIGH);//in3
  digitalWrite(9,LOW); // in4
  delay(1500);
  analogWrite(6,75); //E 2
}
/*if (counter2 == 1){
  analogWrite(6,0);
} */

Serial.println(counter2);
Serial.println(phase);
if (counter2 == 23){
  analogWrite(6,0);
  delay(1000);
  analogWrite(6,75);//E 2
  digitalWrite(8,LOW);//in3
  digitalWrite(9,HIGH); // in4
}

  /* TOP CLAW CLOSING  
  analogWrite(10,250);//E 2
  digitalWrite(11,LOW);//in3
  digitalWrite(12,HIGH); // in4
  delay(4000);
  analogWrite(10,0); //E 2
  delay(5000); */
}
