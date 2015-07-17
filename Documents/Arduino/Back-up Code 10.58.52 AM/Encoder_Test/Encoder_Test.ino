#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Encoder.h>
#include "utility/Adafruit_PWMServoDriver.h"
// -----ENCODER---- //
 const int encoder0PinA = 2;
 const int encoder0PinB = 3;
 int encoder0Pos = 0;
// ----DRIVING---- //
 const int wheel_diameter_cm = 12;
 const int max_speed = 100;
// ----MAPPING---- //
const int room_length = 20;
const int room_width = 20;
const int myMap[room_length][room_width] = {{0, 1},{0, 0}};

 Adafruit_MotorShield AFMS = Adafruit_MotorShield();
 Adafruit_DCMotor *myRightMotor = AFMS.getMotor(1);
 //Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(3);

 void setup() 
 { 


   Serial.begin (9600); //Begin Serial communication at 9600 bps
   Serial.println("Hello");
   AFMS.begin();
   delay(1000);
   pinMode (encoder0PinA, INPUT); //set up Channel A output
   pinMode (encoder0PinB, INPUT); //set up Channel B output
   attachInterrupt(0, doEncoderA, CHANGE);
   attachInterrupt(1, doEncoderB, CHANGE);
   Serial.println("Interrupts attached");  
   Serial.println("Communication begun!");

 } 

 void loop() 
 { 
    //myRightMotor->run(FORWARD);
    //myRightMotor->setSpeed(max_speed);
    //delay(100);
    Serial.println(encoder0Pos);
 }

 void turn(int angle) //0 to 360.  +angle = CW, -angle = CCW
 {
   
 }
 void moveStraight(int distanceCm) //Positive is forward, negative is backward
 {
   int end = encoder0Pos + distanceToEncoderTurns(distanceCm);
   if(distanceCm > 0) {myRightMotor->run(FORWARD); }
   else {myRightMotor->run(BACKWARD);}
   setSpeeds(max_speed, max_speed);
   while (encoder0Pos < end)
   {
     
   }
   setSpeeds(0, 0);
 }
 void setSpeeds(int r, int l) //sets speeds of the motors
 {
  myRightMotor->setSpeed(r);
  //myRightMotor->setSpeed(l);
 }

 int distanceToEncoderTurns(int distance_cm)
 {
    return distance_cm;
 }
 int angleToEncoderTurns(int angle_deg)
 {
  
 }

void doEncoderA(){
Serial.println("A");
 // look for a low-to-high on channel A
 if (digitalRead(encoder0PinA) == HIGH) { 

   // check channel B to see which way encoder is turning
   if (digitalRead(encoder0PinB) == LOW) {  
     encoder0Pos = encoder0Pos + 1;         // CW
   } 
   else {
     encoder0Pos = encoder0Pos - 1;         // CCW
   }
 }

 else   // must be a high-to-low edge on channel A                                       
 { 
   // check channel B to see which way encoder is turning  
   if (digitalRead(encoder0PinB) == HIGH) {   
     encoder0Pos = encoder0Pos + 1;          // CW
   } 
   else {
     encoder0Pos = encoder0Pos - 1;          // CCW
   }
 }

}

void doEncoderB(){
 // look for a low-to-high on channel B
 Serial.println("B");
 if (digitalRead(encoder0PinB) == HIGH) {   

  // check channel A to see which way encoder is turning
   if (digitalRead(encoder0PinA) == HIGH) {  
     encoder0Pos = encoder0Pos + 1;         // CW
   } 
   else {
     encoder0Pos = encoder0Pos - 1;         // CCW
   }
 }

 // Look for a high-to-low on channel B

 else { 
   // check channel B to see which way encoder is turning  
   if (digitalRead(encoder0PinA) == LOW) {   
     encoder0Pos = encoder0Pos + 1;          // CW
   } 
   else {
     encoder0Pos = encoder0Pos - 1;          // CCW
   }
 }

} 

