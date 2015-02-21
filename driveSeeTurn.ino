#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <SharpIR.h>

// CONSTANTS
// These will be compiled away (I think)
// as if they were #define's, but without
// accidentally breaking code.

// IR sensor
const int IRPIN = A2;
const int IRREADINGS = 64;
const int IRDIFFERENCE = 93;
const int IRMODEL = 20150;

// Motor speeds and ramp increment delays.
const int maxSpeed = 20;
const int driveDelay = 2;
const int steerSpeed = 90;
const int steerDelay = 0;

// Distances (cm) and angles (deg) for navigation.
const int sightThresh = 80;  // Distance within which a turn
                             // should be executed rather than a forward run.
//const int backDist = 40;   // Distance to backtrack for very close objects.
const int turnAngle = 44;    // Can be anything, but will executed modulo 22
                             // (the emperical included angle of a
                             // single three-point turn).

const int STEPDELAY = 0;  // Delay within main loop. Lower -> less sanity.

// Sleep switch
const int switchRead = 13;  // Provide digital HIGH ...
const int switchWrite = A0; // ... to be read by the sleep switch
                            // (two pins might not actually be necessary for this).
const int offDelay = 2000;  // Minimum number of ms to be off
                            // before checking the switch again.





// Create IR reader object.
SharpIR sharp(IRPIN, IRREADINGS, IRDIFFERENCE, IRMODEL);

// Create the motor shield object with the default I2C address.
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define two of four motors.
Adafruit_DCMotor *drive = AFMS.getMotor(1);
Adafruit_DCMotor *steering = AFMS.getMotor(2);





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
  pinMode(switchRead, INPUT);
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);

}


bool done = false;
void loop() {
  uint8_t i;
  int dis;

  if(digitalRead(switchRead) == HIGH && done == false)
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
      turn(turnAngle);
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
  }
  else{
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








