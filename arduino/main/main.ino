#include <Encoder.h>

byte inputBuffer[sizeof(int)];
Encoder myEnc(2, 3);
int accel_in = 0;
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("<ready>");
}

long oldPosition  = -999;

void loop() {
  int newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
  }
  if (Serial.available() > 0) {
    size_t num_read = Serial.readBytes(inputBuffer, 2);
    if (num_read == 2) {
      accel_in = *((int*) inputBuffer);
    }
    else {
      accel_in = 0;
    }
    byte* outputBuffer = static_cast<byte*>(static_cast<void*>(&newPosition));
    for (int i = 0; i < sizeof(int); i++) {
      Serial.write(outputBuffer[i]);
    }
  }
}