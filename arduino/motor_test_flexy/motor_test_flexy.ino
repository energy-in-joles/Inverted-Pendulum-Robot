
//      ******************************************************************
//      *                                                                *
//      *       Example demoing how fast a stepper motor can rotate      *
//      *                                                                *
//      *            S. Reifel & Co.                6/24/2018            *
//      *                                                                *
//      ******************************************************************


// The top speed of a stepper motor is related to many factors.  The motor, 
// the load on the motor, the power supply voltage, and this library.
// 
// This library can generate a maximum of about 12,500 steps per second 
// using an Arduino Uno.  Running just one motor in full step mode, with a
// 200 steps per rotation motor, the maximum speed is about 62 RPS or 3750 
// RPM (very few stepper motor can go this fast). Driving one motor in half 
// step mode, a maximum speed of 31 RPS or 1875 RPM can be reached.  In 
// quarter step mode about 15 RPS or 937 RPM.  Running multiple motors at the 
// same time will reduce the maximum speed of each, for example running two 
// motors will reduce the maximum step rate by half or more.
//
// Stepper motors will spin fastest when using a power supply voltage that 
// is many times higher than the motor's voltage rating.  Don't worry about 
// this, the current setting on the driver board keeps the motor protected.  
// I often use NEMA 17 motors with a voltage rating around 3V, then power 
// my driver board with a 24V supply.  Just make sure that your power supply 
// voltage does not exceed the voltage rating of the driver board.  12V  
// motors with a 12V supply will have much lower torque at speed than my  
// 3V/24V combo.  (Note: NEMA 17 describes the size of the motor, 1.7 inches 
// wide)
//
// Smaller motors can typically spin faster than larger ones.  The best way  
// to evaluate a motor is by looking at its "torque curve".  Most of the 
// stepper motors sold by www.pololu.com have a data sheet on their website 
// showing a torque curve (motors sold on Amazon usually do not).
//  
//
// Documentation at:
//         https://github.com/Stan-Reifel/SpeedyStepper
//
//
// The motor must be connected to the Arduino with a driver board having a 
// "Step and Direction" interface.  It's VERY important that you set the 
// motor current first!  Read the driver board's documentation to learn how.

// ***********************************************************************


#include <FlexyStepper.h>


//
// pin assignments
//
const int LED_PIN = 13;
const int MOTOR_STEP_PIN = 4;
const int MOTOR_DIRECTION_PIN = 7;
const int MOTOR_EN_PIN = 8;
int targetPos = 150;


//
// create the stepper motor object
//
FlexyStepper stepper;



void setup() 
{
  //
  // setup the LED pin and enable print statements
  //
  pinMode(LED_PIN, OUTPUT);   
  Serial.begin(9600);
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW);


  //
  // connect and configure the stepper motor to its IO pins
  //
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setCurrentPositionInSteps(0);
  stepper.setTargetPositionInSteps(targetPos);
}



void loop() 
{
  //stepper.setSpeedInStepsPerSecond(7000);
  stepper.setSpeedInStepsPerSecond(1000);
  stepper.setAccelerationInStepsPerSecondPerSecond(20000);
  //stepper.setAccelerationInStepsPerSecondPerSecond(200000);
  while(!stepper.motionComplete())
  {
    stepper.processMovement();
  }
  targetPos = -targetPos;
  stepper.setTargetPositionInSteps(targetPos);
  long currentPosition = stepper.getCurrentPositionInSteps();
  Serial.println(currentPosition);
  delay(10);
}
