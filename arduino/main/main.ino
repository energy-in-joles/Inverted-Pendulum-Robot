#include <Encoder.h>

const int BAUDRATE = 9600;
const String AUTH_STR = "<ready>"; // matching auth string in python script
const int LOOP_BUFFER_SIZE = 1;
const int POS_PER_REV = 2400;

byte inputBuffer[sizeof(int)];
byte loopBuffer[LOOP_BUFFER_SIZE];
byte outputBuffer[sizeof(int) + LOOP_BUFFER_SIZE];

long oldPosition  = -999;

Encoder myEnc(2, 3);

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
  Serial.println(AUTH_STR);
}

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
  }
  EncoderInfo encoderInfo = update_encoder_info(newPosition);

  update_output_buffer(outputBuffer, encoderInfo);

  if (Serial.available() > 0) {
    int accel_in = read_serial();
    update_output_buffer(outputBuffer, encoderInfo);
    for (int i = 0; i < sizeof(outputBuffer); i++) {
      Serial.write(outputBuffer[i]);
    }
  }
}