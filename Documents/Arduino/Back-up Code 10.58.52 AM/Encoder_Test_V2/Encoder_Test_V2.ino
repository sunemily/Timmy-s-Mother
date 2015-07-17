//All distances in Centimeters
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
Encoder encoderRight(2, 3);
Encoder encoderLeft(18, 19);
int max_speed = 128;
int wheelbase_radius = 15;
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myRightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(4);

//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  delay(100);
  AFMS.begin();
  delay(100);
  myRightMotor->run(FORWARD);
  myRightMotor->setSpeed(150);
  myLeftMotor->run(FORWARD);
  myLeftMotor->setSpeed(150);
  encoderRight.write(0); encoderLeft.write(0);
  while (encoderRight.read() < 1000 && encoderLeft.read() < 1000)
  {
    Serial.print(encoderRight.read());
    Serial.print("   ");
    Serial.println(encoderLeft.read());
  }
  myRightMotor->setSpeed(0);
  myLeftMotor->setSpeed(0);
  
}

long positionLeft  = -999;

void loop() {
  long newLeft;
  newLeft = encoderRight.read();
  if (newLeft != positionLeft) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.println();
    positionLeft = newLeft;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    encoderRight.write(0);
  }
}
void moveStraight(int distance) //negative distance is backward, positive is forward
{
  int numTicks = centimetersToTicks(distance);
  encoderRight.write(0); encoderLeft.write(0); //Resets both encoders
  if (distance > 0) {myRightMotor->run(FORWARD); myLeftMotor->run(FORWARD);}
  else {myRightMotor->run(BACKWARD); myLeftMotor->run(BACKWARD);}
  myRightMotor->setSpeed(max_speed); myLeftMotor->setSpeed(max_speed);
  while (encoderRight.read() < numTicks && encoderLeft.read() > -numTicks)
  {
    
  }
  myRightMotor->setSpeed(0); myLeftMotor->setSpeed(0);
}
void turn (int angle) //negative angle is CCW, positive is CW
{
  int numTicks = centimetersToTicks(wheelbase_radius * angle * PI / 180);
  encoderRight.write(0); encoderLeft.write(0);
  if (angle > 0) {myLeftMotor->run(FORWARD); myRightMotor->run(BACKWARD);} 
  else {myLeftMotor->run(BACKWARD); myRightMotor->run(FORWARD);}
  myLeftMotor->setSpeed(max_speed); myRightMotor->setSpeed(max_speed);
  while (encoderRight.read() < numTicks && encoderLeft.read() > -numTicks)
  {
    
  }
  myRightMotor->setSpeed(0); myLeftMotor->setSpeed(0);
}
int centimetersToTicks(int distance)
{
  // return ...
}

