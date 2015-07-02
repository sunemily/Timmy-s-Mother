#include <Adafruit_MotorShield.h>
#include <Servo.h> 

/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/

#include <Wire.h>
//#include < Adafruit_MotorShield.h >
#include "utility/Adafruit_PWMServoDriver.h"
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

int pos = 0;    // variable to store the servo position 
long duration, inches, cm;
const int trigPin = 2;
const int echoPin = 4;
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
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

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
 // pinMode (ir, INPUT);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo
}

void loop() {
  //uint8_t i;
  
  Serial.print("tick");

  sensors();
  

  //for (i=155; i<255; i++) {
   // myMotor->setSpeed(i); 
   // myOtherMotor->setSpeed(i); 
    //delay(100);
  //}
  //for (i=255; i!=155; i--) {
   // myMotor->setSpeed(i);  
   // myOtherMotor->setSpeed(i); 
   // delay(100);
 // }

 
//  turnToAngle(90);
//  delay(1000);
//  turnToAngle(-90);
//  delay(1000);
//  Serial.print("tock");

  //myMotor->run(BACKWARD);
  //myOtherMotor->run(BACKWARD);
  //for (i=155; i<255; i++) {
   // myMotor->setSpeed(i);  
   // myOtherMotor->setSpeed(i);
   // delay(100);
  //}
  //for (i=255; i!=155; i--) {
  //  myMotor->setSpeed(i);  
   // myOtherMotor->setSpeed(i);
   // delay(100);
  //}

 // Serial.print("tech");
  //myMotor->run(RELEASE);
  //myOtherMotor->run(RELEASE);
  //delay(1000);
}
void turnToAngle(int angle)
{
  if (angle > 0)
  {
    myMotor->run(BACKWARD);
    myOtherMotor->run(FORWARD);
  }
  else
  {
    myMotor->run(FORWARD);
    myOtherMotor->run(BACKWARD);
  }
  myMotor->setSpeed(255);
  myOtherMotor->setSpeed(255);
  delay(10.5 * abs(angle));
  myMotor->setSpeed(0);
  myOtherMotor->setSpeed(0);
}

void sensors()
{
  for( pos = 0; pos < 180; pos +=5)
  {
    myservo.write(pos);
      // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);
   Serial.print(cm);
  Serial.println("cm");
    delay(15);        // waits 15ms for the servo to reach the position 
    if(cm<25)
    {
      turnToAngle(pos);
    }else{
      myMotor->run(BACKWARD);
      myMotor->setSpeed(255);
      myOtherMotor->run(BACKWARD);
      myOtherMotor->setSpeed(255);
    }
    
    
  } 

  for(pos =180; pos>= 1; pos-=5)
  {
    myservo.write(pos);
        // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);
   Serial.print(cm);
  Serial.println("cm");
    delay(15);                       // waits 15ms for the servo to reach the position 
    if(cm<25)
    {
      turnToAngle(pos);
    }else{
      myMotor->run(BACKWARD);
      myMotor->setSpeed(255);
      myOtherMotor->run(BACKWARD);
      myOtherMotor->setSpeed(255);
    } 
    
  }
  
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
