#include "wifi_websocket.h"
#include <ServoESP32.h> //Use the Servo librarey for generating PWM

Servo FLESC; //name the servo object, here ESC
Servo FRESC;
Servo RLESC;
Servo RRESC;

const int flEscPin = 25;
const int frEscPin = 26;
const int rlEscPin = 32;
const int rrEscPin = 33;

void setup() {
  Serial.begin(115200);
  connectWifi();
  connectWebsocket();
  FLESC.attach(flEscPin); //Generate PWM in pin 9 of Arduino
  FRESC.attach(frEscPin);
  RLESC.attach(rlEscPin);
  RRESC.attach(rrEscPin);
}

void loop() {
  int* ref = getControlReference();
  Serial.print("throttle: ");
  Serial.print(ref[0]);
  Serial.print(" yaw: ");
  Serial.print(ref[1]);
  Serial.print(" roll: ");
  Serial.print(ref[2]);
  Serial.print(" pitch: ");
  Serial.println(ref[3]);
  FRESC.write(ref[0]+40);
  FLESC.write(ref[0]+40);
  RRESC.write(ref[0]+40);
  RLESC.write(ref[0]+40);
  pollWebsocket();
}
