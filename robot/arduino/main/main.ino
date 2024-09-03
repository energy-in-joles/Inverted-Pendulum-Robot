#include <Encoder.h>
#include <FlexyStepper.h>

const int ENCODER_PIN_A = 2;
const int ENCODER_PIN_B = 3;
const int MOTOR_STEP_PIN = 4;
const int MOTOR_DIR_PIN = 7;
const int MOTOR_EN_PIN = 8;
const int MOTOR_STEP_LIMIT = 200; // 90 DEG limit (800 steps per rev for quarter step setting: 90 DEG == 200 steps)
const int TARGET_POS = 10000; // a large value that the motor will never reach (sets direction for motor spin)
const int MAX_SPEED = 1000;

const int BAUDRATE = 31250;
const String AUTH_STR = "<ready>"; // matching auth string in python script
const int RESET_CMD = 32767;
const int RESET_ENCODER_CMD = 32766;

byte inputBuffer[sizeof(int)];
byte outputBuffer[4];

FlexyStepper stepper;
Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);

// read serial data from python script: 2 byte signed acceleration value
int read_serial() {
  int accel_in = 0;
  size_t num_read = Serial.readBytes(inputBuffer, sizeof(int));
  if (num_read == sizeof(int)) {
    accel_in = *((int*) inputBuffer);
  }
  return accel_in;
}

// prepare output buffer to send position and loop index data to python script
// buffer: 22 bits for encoder position (little endian), 12 bits for stepper pos
void update_output_buffer(byte *outputBuffer, long encoderPos, int currentStepperPos) {
  outputBuffer[0] = (encoderPos >> 0) & 0xFF;
  outputBuffer[1] = (encoderPos >> 8) & 0xFF;
  outputBuffer[2] = (encoderPos >> 14) & 0xFC | (currentStepperPos >> 10) & 0x03;
  outputBuffer[3] = (currentStepperPos >> 0) &0xFF;
}

void reset_pos() {
  stepper.setAccelerationInStepsPerSecondPerSecond(20000);
  stepper.moveToPositionInSteps(0);
}

void setup() {
  Serial.begin(BAUDRATE);
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW);

  // connect and configure the stepper motor to its IO pins
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIR_PIN);
  stepper.setCurrentPositionInSteps(0);
  stepper.setSpeedInStepsPerSecond(MAX_SPEED);
  delay(10);
  Serial.println(AUTH_STR);
}

int accel_in = 0;
bool isResting = true; // prevent motor from running at start without any input
bool isGoingRight = false;
int currentStepperPos = 0;

void loop() {
  if (Serial.available() > 0) {
    accel_in = read_serial();
    // reset stepper back to start position
    if (accel_in == RESET_CMD) {
      reset_pos();
      isResting = true;
      accel_in = 0;
      Serial.println(AUTH_STR);
      return;
    } 
    // reset encoder value when close to overflow
    if (accel_in == RESET_ENCODER_CMD) {
      isResting = true;
      accel_in = 0;
      myEnc.write(0);
      return;
    }

    long newPosition = myEnc.read();
    // update python script with positional information
    update_output_buffer(outputBuffer, newPosition, currentStepperPos);
    for (int i = 0; i < sizeof(outputBuffer); i++) {
      Serial.write(outputBuffer[i]);
    }
  }

  // move motor and update motor movement information
  if (abs(accel_in) > 0) {
    isResting = false;
  }

  if (accel_in < 0 || accel_in == 0 && !isGoingRight) {
    stepper.setTargetPositionInSteps(-TARGET_POS);
    isGoingRight = false;
  }
  else {
    stepper.setTargetPositionInSteps(TARGET_POS);
    isGoingRight = true;
  }
  // update motor movement using accel_in value
  stepper.setAccelerationInStepsPerSecondPerSecond(abs(accel_in));

  currentStepperPos = stepper.getCurrentPositionInSteps();
  if (currentStepperPos <= MOTOR_STEP_LIMIT && currentStepperPos >= -MOTOR_STEP_LIMIT && !isResting) {
    stepper.processMovement();
  }
}