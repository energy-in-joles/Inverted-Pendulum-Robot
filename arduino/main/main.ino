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

const int BAUDRATE = 9600;
const String AUTH_STR = "<ready>"; // matching auth string in python script
const int LOOP_BUFFER_SIZE = 1;
const int POS_PER_REV = 2400;

byte inputBuffer[sizeof(int)];
byte loopBuffer[LOOP_BUFFER_SIZE];
byte outputBuffer[sizeof(int) + LOOP_BUFFER_SIZE];

FlexyStepper stepper;
Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);

struct EncoderInfo {
  int pos;
  int loop_i; // store no. of full loops made (start index: 0, range: -127 to 128)
};

// store raw long position value as int position (0 to 2399 -> clockwise) and loop_i (-127 to 128)
EncoderInfo update_encoder_info(long newPosition) {
    EncoderInfo encoderInfo;

    // Calculate position within one revolution
    encoderInfo.pos = ((newPosition % POS_PER_REV) + POS_PER_REV) % POS_PER_REV;

    // Calculate the number of full revolutions
    encoderInfo.loop_i = newPosition / POS_PER_REV;

    // Handle negative values to ensure proper loop index calculation
    if (newPosition < 0 && newPosition % POS_PER_REV != 0) {
        encoderInfo.loop_i--;
    }

    return encoderInfo;
}

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
void update_output_buffer(byte *outputBuffer, EncoderInfo encoderInfo) {
  outputBuffer[0] = (encoderInfo.pos >> 0) & 0xFF;
  outputBuffer[1] = (encoderInfo.pos >> 8) & 0xFF;
  outputBuffer[2] = (encoderInfo.loop_i >> 0) &0xFF;
}

void setup() {
  Serial.begin(BAUDRATE);
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW);

  // connect and configure the stepper motor to its IO pins
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIR_PIN);
  stepper.setCurrentPositionInSteps(0);
  stepper.setSpeedInStepsPerSecond(MAX_SPEED);

  Serial.println(AUTH_STR);
}

long oldPosition  = -999;
int accel_in = 0;
int isResting = true; // prevent motor from running at start without any input

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
  }
  EncoderInfo encoderInfo = update_encoder_info(newPosition);

  update_output_buffer(outputBuffer, encoderInfo);

  if (Serial.available() > 0) {
    accel_in = read_serial();
    update_output_buffer(outputBuffer, encoderInfo);
    for (int i = 0; i < sizeof(outputBuffer); i++) {
      Serial.write(outputBuffer[i]);
    }
  }

  if (abs(accel_in) > 0) {
    isResting = false;
  }

  if (accel_in < 0) {
    stepper.setTargetPositionInSteps(-TARGET_POS);
  }
  else {
    stepper.setTargetPositionInSteps(TARGET_POS);
  }
  // update motor movement using accel_in value
  stepper.setAccelerationInStepsPerSecondPerSecond(abs(accel_in));

  long currentPos = stepper.getCurrentPositionInSteps();
  if (currentPos <= MOTOR_STEP_LIMIT && currentPos >= -MOTOR_STEP_LIMIT && !isResting) {
    stepper.processMovement();
  }
}