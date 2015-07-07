#include <Wire.h> 
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <I2C.h>
#include "utility/Adafruit_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myRightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(1);
Servo myServo;
const int servoPin = 9;
//const int trigPin = 2;
//const int echoPin = 6;
char LIDARLite_ADDRESS = 0x62;

void setup() {
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  I2c.begin(); delay(100); I2c.timeOut(50);
  myServo.attach(servoPin);
}
void loop() {
  delay(5000);
  myServo.write(0);
  delay(1000);
  //for (int i = 0; i <= 180; i+= 1) {myServo.write(i); Serial.print(llGetDistance()); Serial.print(", "); Serial.println(i); delay(60);}
  Serial.println("Going to closest wall");
  goToClosestWall();
  Serial.println("Finished closest wall.  Servo at 180");
  int startDistance = llGetDistanceAverage(5);
  int maxSpeed = 155;
  int difference = 0;
  int buffer; int bufferFactor = 10;
  moveForward(maxSpeed);
  Serial.println("Motors forward");
   //Version 1:
  while (1) //run this forever
  {
    //Serial.println(difference);
    delay(10);
    difference = startDistance - llGetDistanceAverage(5); //update difference
    buffer = abs(difference);
    if (difference < -5) //if the robot is too far away from the wall...
    {
       turnWithGivenSpeeds(maxSpeed - buffer*bufferFactor, maxSpeed);
      //Serial.println("First");
    }
    else if (difference > 5) //if the robot is too close to the wall
    {
      turnWithGivenSpeeds(maxSpeed, maxSpeed - buffer*bufferFactor);
       //Serial.println("Second");
    }
    else //do this as the default
    {
      moveForward(maxSpeed);
      //Serial.println("Third");   
    }
  }
  
  //Version 2:
  /*
  int xPos = 0; int yPos = 0; int angle = 90;
  int startTime = millis();
  while (1)
  {
    
  }
  */
  
}

void goToClosestWall(){
  minAngle = getAngleWithMinDistance(5);
  if(minAngle < 90){
    turnRight(90-minAngle);
  }
  else{
    turnLeft(minAngle-90);
  }
  distToWall = llGetDistance();
  moveForward(100);
  while(llGetDistance() > 15){
    delay(10);
  }
  turnRightInPlace(90);
}

void moveForward(int distance, int speed)
{
  
}
void moveBackward(int distance, int speed)
{
  
}
void moveForward(int speed) //moves the robot forward at a given speed.
{
  myLeftMotor->setSpeed(speed);
  myRightMotor->setSpeed(speed);
  myLeftMotor->run(FORWARD);
  myRightMotor->run(FORWARD);
}
void moveBackward(int speed) //moves the robot backward at a given speed.
{
  myLeftMotor->setSpeed(speed);
  myRightMotor->setSpeed(speed);
  myLeftMotor->run(BACKWARD);
  myRightMotor->run(BACKWARD);
}

void turnRightInPlace(int angle) // turns to the right about the center of the wheels of the robot.
{
    myRightMotor->setSpeed(255);
    myLeftMotor->setSpeed(255);
    myRightMotor->run(BACKWARD);
    myLeftMotor->run(FORWARD);
    delay(10.5 * abs(angle));
    myRightMotor->setSpeed(0);
    myLeftMotor->setSpeed(0);
}
void turnLeftInPlace(int angle) // turns to the left about the center of the wheels of the robot.
{
    myRightMotor->setSpeed(255);
    myLeftMotor->setSpeed(255);
    myRightMotor->run(FORWARD);
    myLeftMotor->run(BACKWARD);
    delay(10.5 * abs(angle));
    myRightMotor->setSpeed(0);
    myLeftMotor->setSpeed(0);
}
void turnLeft(int angle){ //turns to the left about the left wheel
    myRightMotor->setSpeed(255);
    myLeftMotor->setSpeed(0);
    myRightMotor->run(FORWARD);
    delay(5 * abs(angle));
    myRightMotor->setSpeed(0);
}
void turnRight(int angle) //turns to the right about the right wheel.
{
    myLeftMotor->setSpeed(255);
    myRightMotor->setSpeed(0);
    myLeftMotor->run(FORWARD);
    delay(5 * abs(angle));
    myLeftMotor->setSpeed(0);
}
void turnWithGivenSpeeds(int leftSpeed, int rightSpeed) //turn each motor at its specified speed.
{
  myLeftMotor->run(FORWARD);
  myRightMotor->run(FORWARD);
  myLeftMotor->setSpeed(leftSpeed);
  myRightMotor->setSpeed(rightSpeed);
  Serial.println(leftSpeed);
  Serial.println(rightSpeed);
  delay(5000);
}
void stop() //stops both motors
{
  myRightMotor->run(RELEASE);
  myLeftMotor->run(RELEASE);
}


int getAngleWithMinDistance(int numReadings)
{
  int increment = 180 / numReadings;
  int minD = llGetDistance(); 
  int minAngle = 0; 
  int val;
  for (int i = increment; i <= 180; i += increment)
  {
    myServo.write(i);
    val= llGetDistance();
    
    if (val < minD) {
      minD = val; 
      minAngle = i;
    } 
    Serial.print(i);
    Serial.print(", ");
    Serial.println(val);
  }
  Serial.print("Min angle is ");
  Serial.println(minAngle);
  return minAngle;
}
// Write a register and wait until it responds with success
void llWriteAndWait(char myAddress, char myValue){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,myAddress, myValue); // Write to LIDAR-Lite Address with Value
    delay(2); // Wait 2 ms to prevent overpolling
  }
}

// Read 1-2 bytes from a register and wait until it responds with sucess
byte llReadAndWait(char myAddress, int numOfBytes, byte arrayToSave[2]){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until success message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,myAddress, numOfBytes, arrayToSave); // Read 1-2 Bytes from LIDAR-Lite Address and store in array
    delay(2); // Wait 2 ms to prevent overpolling
  }
  return arrayToSave[2]; // Return array for use in other functions
}



/* ==========================================================================================================================================
Get 2-byte distance from sensor and combine into single 16-bit int
=============================================================================================================================================*/

int llGetDistance(){
  llWriteAndWait(0x00,0x04); // Write 0x04 to register 0x00 to start getting distance readings
  byte myArray[2]; // array to store bytes from read function
  llReadAndWait(0x8f,2,myArray); // Read 2 bytes from 0x8f
  int distance = (myArray[0] << 8) + myArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  return(distance);
}

/* ==========================================================================================================================================
Get raw velocity readings from sensor and convert to signed int
=============================================================================================================================================*/

int llGetVelocity(){
  llWriteAndWait(0x00,0x04); // Write 0x04 to register 0x00 to start getting distance readings
  llWriteAndWait(0x04,0x80); // Write 0x80 to 0x04 to switch on velocity mode 
  byte myArray[1]; // Array to store bytes from read function
  llReadAndWait(0x09,1,myArray); // Read 1 byte from register 0x09 to get velocity measurement 
  return((int)((char)myArray[0])); // Convert 1 byte to char and then to int to get signed int value for velocity measurement
}


/* ==========================================================================================================================================
Average readings from velocity and distance
int numberOfReadings - the number of readings you want to average (0-9 are possible, 2-9 are reccomended)
=============================================================================================================================================*/


int llGetDistanceAverage(int numberOfReadings){ 
  if(numberOfReadings < 2){
    numberOfReadings = 2; // If the number of readings to be taken is less than 2, default to 2 readings
  }
  int sum = 0; // Variable to store sum
  for(int i = 0; i < numberOfReadings; i++){ 
      sum = sum + llGetDistance(); // Add up all of the readings
  }
  sum = sum/numberOfReadings; // Divide the total by the number of readings to get the average
  return(sum);
}

int llGetVelocityAverage(int numberOfReadings){ 
  int sum = 0; // Variable to store sum
  for(int i = 0; i < numberOfReadings; i++){ 
      sum = sum + llGetVelocity(); // Add up all of the readings
  }
  sum = sum/numberOfReadings; // Divide the total by the number of readings to get the average
  return(sum);
}

