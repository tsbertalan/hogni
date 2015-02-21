/* 
For use witfh the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <SharpIR.h>


#define IRPIN A2
#define IRREADINGS 64
#define IRDIFFERENCE 93
#define IRMODEL 20150
SharpIR sharp(IRPIN, IRREADINGS, IRDIFFERENCE, IRMODEL);



// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 
her 
// Define two of four motors
Adafruit_DCMotor *drive = AFMS.getMotor(1);
Adafruit_DCMotor *steering = AFMS.getMotor(2);




  
const int maxSpeed = 20;
const int driveDelay = 2;
const int steerSpeed = 90;
const int steerDelay = 0;
  
const int sightThresh = 80;
const int backDist = 40;
  
const int STEPDELAY = 0;

const int switchPin = 13;
const int switchOut = A0;
const int offDelay = 2000; // Minimum number of ms to be off






void rampUp(Adafruit_DCMotor *motor, int topSpeed, int delayTime){
  uint8_t i;
  for(i=0; i<topSpeed; i++){
    motor->setSpeed(i);
    delay(delayTime);
  }
}

void rampDown(Adafruit_DCMotor *motor, int topSpeed, int delayTime){
  uint8_t i;
  for(i=topSpeed; i!=0; i--){
    motor->setSpeed(i);
    delay(delayTime);
  }
}




void notImplemented(String caller)
{
  Serial.print("NOT IMPLEMENTED:");
  Serial.println(caller);
}










void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  //AFMS.begin();  // create with the default frequency 1.6KHz
  AFMS.begin(10000);  // OR with a different frequency
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  drive->setSpeed(150);
  drive->run(FORWARD);
  // turn on motor
  drive->run(RELEASE);
  steering->run(RELEASE);
  
  Serial.begin(9600);
  pinMode(IRPIN, INPUT);
  pinMode(switchPin, INPUT);
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);

}


void turn22L(){
  // STEER LEFT
  steering->run(FORWARD);
  rampUp(steering, steerSpeed, steerDelay);
  // GO FORWARD
  drive->run(FORWARD);
  rampUp(drive, maxSpeed, driveDelay);
  delay(500);
  rampDown(drive, maxSpeed, driveDelay);
  //STEER STRAIGHT
  rampDown(steering, steerSpeed, steerDelay);
  //STEER RIGHT
  steering->run(BACKWARD);
  rampUp(steering, steerSpeed, steerDelay);
  // GO BACK
  drive->run(BACKWARD);
  rampUp(drive, maxSpeed, driveDelay);
  delay(500);
  rampDown(drive, maxSpeed, driveDelay);
  //STEER STRAIGHT
  rampDown(steering, steerSpeed, steerDelay);
}



void turn(int angle)
{
  String msg = "";
  Serial.println(msg + "Turning " + angle + " degrees.");
  if(angle > 0)
  {
    // Turning left.
    for(int i=0; i<angle / 22; i++)
    {
      turn22L();
    }
  }
  else
  {
    Serial.println("I can only turn left. I'm not an ambi-turner.");
  }
}




void go(int cmDist, bool forward)
{
  String msg = "";
  Serial.print("Going ");
  if(forward){
    Serial.print(" forward ");
  }else{
    Serial.print(" backward ");
  }
  Serial.println(msg + cmDist + " cm.");
  
  //drive->run(RELEASE); // stop the motor
  if(forward)
  {
    drive->run(FORWARD);
  }
  else
  {
    drive->run(BACKWARD);
  }
  rampUp(drive, maxSpeed, driveDelay);
  delay(1000 * cmDist / 50); // 1000 ms is about right for 50 cm at this speed. Very fragile, I know.
  rampDown(drive, maxSpeed, driveDelay);
}



void stopDrive()
{
  Serial.println("Stopping.");
  drive->run(RELEASE);
}



void releaseSteering()
{
  Serial.println("Centering steering.");
  steering->run(RELEASE);
}





int getDistance()
{
  int dist = sharp.distance();
  String msg = " ";
  Serial.println(msg + dist + "cm ");
  return dist;
}







bool done = false;
void loop() {
  uint8_t i;
  int dis;

  if(digitalRead(switchPin) == HIGH && done == false)
  {
    dis = getDistance();
     
    if(dis < sightThresh)
    {
      // If an object is ahead of us, turn 90 degrees.
//      if(dis < backDist)
//      {
//        // If the object is very close, go back a little first, then turn.
//        go(25, false);
//      }
      turn(44);
    }
    else
    {
      // Else, run forward.
      if(dis > 0)
      {
        go(dis / 4, true);
      }
    }
    
    delay(STEPDELAY);
//    go(100, true);
//    done = true;
  }
  else
  { // switch is off
    stopDrive();
    releaseSteering();
    delay(offDelay);
  }
}

