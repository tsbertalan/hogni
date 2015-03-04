#include <Wire.h>
#include <Adafruit_MotorShield.h>
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
const int maxSpeed = 64;
int driveSpeed = 40;  // 20 at 12v
const int driveDelay = 2;
const int steerSpeed = 196; // 90 at 12v
const int steerDelay = 0;

// Distances (cm) and angles (deg) for navigation.
const int sightThresh = 80;  // Distance within which a turn
                             // should be executed rather than a forward run.
//const int backDist = 40;   // Distance to backtrack for very close objects.
const int turnAngle = 22;    // Can be anything, but will executed modulo 22
                             // (the emperical included angle of a
                             // single three-point turn).

const int STEPDELAY = 0;  // Delay within main loop. Lower -> less sanity.

const int offDelay = 2000;  // Minimum number of ms to be off
                            // before checking the switch again.





// Create IR sensor object.
SharpIR irSensor(irPin, irReadings, irDifference, irModel);

// Create the motor shield object with the default I2C address.
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define two of four motors.
Adafruit_DCMotor *drive = AFMS.getMotor(1);
Adafruit_DCMotor *steering = AFMS.getMotor(2);


const int leftSwitch = 6;
const int rightSwitch = 3;
const int potPin = 3;


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
  pinMode(leftSwitch, INPUT);  // Without a restistor (LED counts, right?) beside the input pin, we'd need INPUT_PULLUP
  pinMode(rightSwitch, INPUT);

}

int potVal = 0;
void loop()
{
  Serial.println("begin.");
  potVal = analogRead(potPin);  // [0, 1023]
  Serial.print("potVal: ");
  Serial.println(potVal);
  driveSpeed = int(potVal * float(maxSpeed) / 1023.0);
  Serial.print("driveSpeed: ");
  Serial.println(driveSpeed);  
  int dist;  // Distance seen by forward-facing IR, in cm.
  
  if (digitalRead(leftSwitch) == HIGH)
  {
    if (digitalRead(rightSwitch) == HIGH)
    {
      Serial.println("LEFT  RIGHT");
      go(-32);
    }
    else
    {
      Serial.println("LEFT");
      turn(45);
    }
  }
  else
  {
    if (digitalRead(rightSwitch) == HIGH)
    {
      Serial.println("      RIGHT");
      turn(-45);
    }
    else
    {
      // Neither switch is closed.

  
  
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

    }
  }
  delay(STEPDELAY);
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

void cutRight()
{
    steering->run(BACKWARD);
    rampUp(steering, steerSpeed, steerDelay);
}

void cutLeft()
{
    steering->run(FORWARD);
    rampUp(steering, steerSpeed, steerDelay);
}

void turn22R()
{
  cutLeft();
  go(-25);
  releaseSteering();
  cutRight();
  go(25);
  releaseSteering();
}

void turn22L()
{
  cutRight();
  go(-25);
  releaseSteering();
  cutLeft();
  go(25);
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
    // Turning right.
    for(int i=0; i<abs(angle) / 22; i++)
    {
      turn22R();
    }
  }
}






void go(int dist)
{
  // Proceed dist cm forward. dist can be negative.
  if(dist < 0)
  {
    go(abs(dist), false);
  }
  else
  {
    go(dist, true);
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
  rampUp(drive, driveSpeed, driveDelay);
  delay(1000 * dist / 50);
  rampDown(drive, driveSpeed, driveDelay);
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
  // Query the IR sensor for forward distance. Accept no negative readings.
  
  int dist = -1;
  while(dist < 0){
    dist = irSensor.distance();
    String msg = " ";
    Serial.println(msg + dist + "cm ");
    delay(1);
  }
  return dist;
}

// (No consideration has yet been given to
// the servo on which the IR sensor is mounted.) 






