#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include "utility/Adafruit_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(2);

 const int encoder0PinA = 18;
 const int encoder0PinB = 19;
 volatile int encoder0Pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(500);
  Serial.println("Serial communication begun");
  delay(100);
  AFMS.begin();
  Serial.println("MotorShield communication begun");
  myMotor->run(FORWARD);
  Serial.println("Motor initialized");
  myMotor->setSpeed(255);
  delay(1000);
  myMotor->run(RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:
  myMotor->run(BACKWARD);
  myMotor->setSpeed(50);
  Serial.println(digitalRead(encoder0PinA));
  Serial.println(digitalRead(encoder0PinB));


}
