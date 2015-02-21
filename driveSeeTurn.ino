#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <SharpIR.h>

// CONSTANTS
// These will be compiled away (I think)
// as if they were #define's, but without
// accidentally breaking code.

// IR sensor
const int irPin = A2;
const int irReadings = 64;
const int irDifference = 93;
const int irModel = 20150;

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





// Create IR sensor object.
SharpIR irSensor(irPin, irReadings, irDifference, irModel);

// Create the motor shield object with the default I2C address.
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define two of four motors.
Adafruit_DCMotor *drive = AFMS.getMotor(1);
Adafruit_DCMotor *steering = AFMS.getMotor(2);





void setup()
{
  // Set up Serial library at 9600 bps for monitoring prints.
  Serial.begin(9600);

  //AFMS.begin();  // create with the default frequency 1.6KHz
  AFMS.begin(10000);  // OR with a different frequency

  // Motors should be off at start.
  stopDrive();
  releaseSteering();

  
  // Set pinmodes.
  pinMode(irPin, INPUT);
  pinMode(switchRead, INPUT);
  pinMode(A0, OUTPUT);
  
  // Turn on pin for off switch.
  digitalWrite(A0, HIGH);
}


void loop()
{
  uint8_t i;
  int dist;  // Distance seen by forward-facing IR, in cm.
  
  if(digitalRead(switchRead) == HIGH) // Check sleep switch.
  {
    dist = getDistance();

    if(dist < sightThresh)
    {
      // If an object is ahead of us, turn before proceeding.
      //      if(dis < backDist)
      //      {
      //        // If the object is very close,
      //        // go back a little first, then turn.
      //        go(25, false);
      //      }
      turn(turnAngle);    
    }
    else
    { // Clear ahead; run forward.
      if(dist > 0)
      {
        go(dist / 4, true);
      }   
    }
    delay(STEPDELAY);
    
  }
  else
  { // switch is off
  
    stopDrive();
    releaseSteering();
    delay(offDelay);
    
  }
}





void rampUp(Adafruit_DCMotor *motor, int topSpeed, int delayTime)
{
  // Bring motor up to speed topSpeed from zero,
  // in topSpeed increments, with a delay of delayTime between increments.
  
  uint8_t i;
  for(i=0; i<topSpeed; i++){
    motor->setSpeed(i);
    delay(delayTime);
  }
}


void rampDown(Adafruit_DCMotor *motor, int topSpeed, int delayTime)
{ 
  // Bring motor down from speed topSpeed to zero,
  // in topSpeed increments, with a delay of delayTime between increments.
  uint8_t i;
  for(i=topSpeed; i!=0; i--){
    motor->setSpeed(i);
    delay(delayTime);
  }
}


void turn22L()
{ 
  // Do a 22-degree three-point turn.
  // (That's what I measured; this code is super fragile.)
  
  // STEER LEFT
  steering->run(FORWARD);
  rampUp(steering, steerSpeed, steerDelay);
  // GO FORWARD
  drive->run(FORWARD);
  rampUp(drive, maxSpeed, driveDelay);
  delay(500);
  rampDown(drive, maxSpeed, driveDelay);
  //STEER STRAIGHT
  releaseSteering();
  //STEER RIGHT
  steering->run(BACKWARD);
  rampUp(steering, steerSpeed, steerDelay);
  // GO BACK
  drive->run(BACKWARD);
  rampUp(drive, maxSpeed, driveDelay);
  delay(500);
  rampDown(drive, maxSpeed, driveDelay);
  //STEER STRAIGHT
  releaseSteering();
}


void turn(int angle)
{
  // Assemble three-point turns into a rotation of floor(angle/22) degrees.
  
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
    // Who would ever want to be able to turn right?
    Serial.println("I can only turn left. I'm not an ambi-turner.");
  }
}


void go(int dist, bool goForward)
{
  // Proceed dist cm forward or backward.
  
  String msg = "";
  Serial.print("Going ");
  if(goForward){
    Serial.print(" forward ");
  }
  else{
    Serial.print(" backward ");
  }
  Serial.println(msg + dist + " cm.");

  if(goForward)
  {
    drive->run(FORWARD);
  }
  else
  {
    drive->run(BACKWARD);
  }
  rampUp(drive, maxSpeed, driveDelay);
  delay(1000 * dist / 50); // 1000 ms is about right for 50 cm at this speed. Very fragile, I know.
  rampDown(drive, maxSpeed, driveDelay);
}


void stopDrive()
{
  // Hammertime
  
  Serial.println("Stopping.");
  drive->run(RELEASE);
}



void releaseSteering()
{
  // Turn off the steering motor.
  
  Serial.println("Centering steering.");
  steering->run(RELEASE);
}


int getDistance()
{
  // Query the IR sensor for forward distance.
  
  int dist = irSensor.distance();
  String msg = " ";
  Serial.println(msg + dist + "cm ");
  return dist;
}

// (No consideration has yet been given to
// the servo on which the IR sensor is mounted.) 






