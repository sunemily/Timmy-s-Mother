#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define trigPin 11
#define echoPin 13

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(2);
// You can also make another motor on port M2
Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(3);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  //Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

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
  //uint8_t i;
  
  //Ultrasonic
  long duration, distance;
  

  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

  Serial.print(distance);
  Serial.println(" cm");

    if (distance < 25) { 
      turnToAngle(90);
    } else {
      myMotor->run(BACKWARD);
      myMotor->setSpeed(255);
      myOtherMotor->run(BACKWARD);
      myOtherMotor->setSpeed(255);
    }

/** Motor ramp test 

  for (i=155; i<255; i++) {
    myMotor->setSpeed(i); 
    myOtherMotor->setSpeed(i); 
    delay(100);
  }
  for (i=255; i!=155; i--) {
    myMotor->setSpeed(i);  
    myOtherMotor->setSpeed(i); 
    delay(100);
  }
  

  myMotor->run(BACKWARD);
  myOtherMotor->run(BACKWARD);
  
  for (i=155; i<255; i++) {
    myMotor->setSpeed(i);  
    myOtherMotor->setSpeed(i);
    delay(100);
   }
   
  for (i=255; i!=155; i--) {
    myMotor->setSpeed(i);  
    myOtherMotor->setSpeed(i);
    delay(100);
   }
   **/

   /** John's 90 degree turn 
  turnToAngle(90);
  delay(1000);
  turnToAngle(-90);
  delay(1000);
  Serial.print("tock");
  **/

  //Serial.print("tech");
  //myMotor->run(RELEASE);
  //myOtherMotor->run(RELEASE);

  distance = 100;
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

