#include <Adafruit_MotorShield.h>

#include <SoftwareSerial.h>
#include <Servo.h>
//#include <SharpIR.h>
#include <Wire.h>

#include "utility/Adafruit_PWMServoDriver.h"

//For sensing from the front
const int frontTrigPin = 2;
const int frontEchoPin = 6; 

//#define ir A0 //the pin where the sensor is attached??
//#define model 1080 //model: 1080 for GP2Y0A21Y, 20150 for GP2Y0A02Y

//SharpIR sharp(ir, 25, 93, model);

//Servo object to control servo
Servo myservo;
const int servoPin = 9;

int pos = 0; //pos=0 means front sensor will point to the front, back to the back

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(3);

long distance, duration;

void setup() {
  // put your setup code here, to run once:
  
  delay(2000); // let robot initialize

  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);


  //(ir, INPUT);
  //pinMode(ir2, INPUT);

  //sensor code also has a Serial.begin(9600);
  myservo.attach(servoPin);

   AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(255);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);

  myOtherMotor->setSpeed(255);
  myOtherMotor->run(FORWARD);
  // turn on motor
  myOtherMotor->run(RELEASE);

}

void loop() {

  myservo.write(90);
  delay(3000);
  goToCloseWall(315);
  turnRight(945); //turn 90 degrees when at closest wall
  //circumnavigate(25);
  
  while(1);
  
}

/**
//takes in the max distance from the wall that the 
//device should go
//this assumes all walls are 90 degrees
void circumnavigate(int dist){
  //must look at these conditions
  while (distFromLeftWall() < dist && cmToWall() > 15){
    moveForward();
  }
  if(distFromFrontWall() < distance){
    turnRight(945); 
  }
  else{
    turnLeft(945);
  }
  circumnavigate(dist);
}
**/

/*Rotates at most 360 degrees 
  and checks the distance to the wall every 
  del delay of rotating.  
  Rotates back to the angle which measured 
  shortest distance to the wall and goes 
  forward to that wall.
  Assumption: 1520 delay is 360 degrees*/
void goToCloseWall(int del){
  
  //Find angle that provides min distance
  int i = 0;
  int minDis = cmToWall();  
  int minSec = 0;
  while(i < 3780){ //go around in circle
    turnRight(del);
    i+=del;
    stopR();
    int check = cmToWall();
    if(check < minDis){
      minDis = check;
      minSec = i;
    }
  }
  
  //rotate back to minimum distance angle
  turnLeft(i-minSec);
  moveForward();
  
  //need to move minDis to get to wall
  //Velocity= 50cm/s
  delay(minDis*20); 
}

void stopR(){
  myMotor->run(BACKWARD);
  myOtherMotor->run(BACKWARD);
  myMotor->setSpeed(0);  
    myOtherMotor->setSpeed(0);
 
}

void moveForward(){
myMotor->run(BACKWARD);
  myOtherMotor->run(BACKWARD);
  myMotor->setSpeed(255);  
    myOtherMotor->setSpeed(255);
}

void turnRight(int del){


    myMotor->run(FORWARD);
    myOtherMotor->run(BACKWARD);
 
  
  myMotor->setSpeed(255);
  myOtherMotor->setSpeed(255);

 

  delay(del);
  myMotor->setSpeed(0);
  myOtherMotor->setSpeed(0);
}


void turnLeft(int del){
 myMotor->run(BACKWARD);
    myOtherMotor->run(FORWARD);
  
  myMotor->setSpeed(255);
  myOtherMotor->setSpeed(255);

  delay(del);
  
  myMotor->setSpeed(0);
  myOtherMotor->setSpeed(0);
}

void turnToAngle(int angle)
{
  if (angle > 0) {
    myMotor->run(BACKWARD);
    myOtherMotor->run(FORWARD);
  } else {
    myMotor->run(FORWARD);
    myOtherMotor->run(BACKWARD);
  }
  
  myMotor->setSpeed(255);
  myOtherMotor->setSpeed(255);
  delay(10.5 * abs(angle));
  myMotor->setSpeed(0);
  myOtherMotor->setSpeed(0);
}


//if the decimal points are really different, I don't think it
//is worth checking in another function.
int cmToWall(){
  myservo.write(90); //face forward
  delay(100);
  digitalWrite(frontTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(frontTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontTrigPin, LOW);

  duration = pulseIn(frontEchoPin, HIGH);

  //do we need a delay?
  //delay(1000);

  distance = duration/59; //converstion from ms to cm
  return distance;

}

