/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <Adafruit_MotorShield.h>


Encoder myEnc(2, 4);


// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);





void setup() {
  Serial.begin(9600);
 
  Serial.println("Basic Encoder Test:");
  AFMS.begin();
  myMotor->setSpeed(0);
  myMotor->run(FORWARD);
  // turn on motor
}

long oldPosition = -999;
int frequency = 100;
int period = 2000;
int timeStamp = 0;
int timeStamp2 = 0;
int command=0;
bool dir = true;

int sp = 0;

// For 12V with cables: no max intg, kp = 
void loop() {
  //encoder
  if (millis()>timeStamp+frequency){
    timeStamp= millis();
    
    command=getCommand(sp);
    if(command > 255){
      command = 255;
    }else if(command < -255){
      
      command = -255;
      
    }
    if(command < 0 && dir)
      {
        myMotor->run(BACKWARD);
        dir = false;
    }else if(command > 0 && !dir){
        dir = true;
        myMotor->run(FORWARD);
    }
//encoder
    if(command < 0){
      command = abs(command);
    }



    Serial.println(findError(sp));
    Serial.println(command);
  }

  //servo motor
  if (millis()>timeStamp+period){
    timeStamp2 = millis();
 
  }
    //myMotor->run(FORWARD);
    myMotor->setSpeed(command);
} // servo motor

int getEncoderCount(){
  // Return ENCODER COUNT!!!!!
  return myEnc.read();
}



double findError(int setP){
  return setP - getEncoderCount();
}

double kp = 2;
double getProportionalCommand(double err){
  return kp*err;
}
double ki = 0.001;
double integral = 0.000;
double maxIntegral = 0;
double getIntegralControl(double err){
  integral += err;
  if(abs(integral) >abs( maxIntegral)){
    if(integral<0){
        //integral= -1*maxIntegral;
    }else{
      //integral=maxIntegral;
    }
  }
  return ki*err;

}

double getCommand(double setP){
  double err = findError(setP);
  double p = getProportionalCommand(err);
  double i = getIntegralControl(err);

  return p+i;
}