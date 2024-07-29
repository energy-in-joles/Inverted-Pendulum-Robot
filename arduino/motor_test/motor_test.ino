// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections
const int dirPin = 7;
const int stepPin = 4;
 int enPin = 8;
 int n = 0;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  pinMode( enPin ,OUTPUT);
  digitalWrite( enPin , LOW);
  Serial.begin(9600);

 // myStepper.setCurrent(0);
  
 // myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(20000);
  myStepper.setMaxSpeed(4000);
  //myStepper.setSpeed(1000);
  myStepper.moveTo(150);
  
}

void loop() {
  Serial.println(myStepper.currentPosition());
  if (myStepper.distanceToGo() == 0) 
    myStepper.moveTo(-myStepper.currentPosition());
  myStepper.run();
}