/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

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
}

void loop() {
  uint8_t i;
  
  Serial.print("tick");

  myMotor->run(FORWARD);
  myOtherMotor->run(FORWARD);
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
  
  Serial.print("tock");

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

  Serial.print("tech");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
}
void turnToAngle(int angle)
{
  myMotor->setSpeed(255);
  myOtherMotor->setSpeed(-255);
  delay(1000 * 11 /(6*4*51) * angle);
}

