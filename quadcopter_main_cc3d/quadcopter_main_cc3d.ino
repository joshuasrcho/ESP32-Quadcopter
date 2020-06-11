#include "wifi_websocket.h"

#include <WiFi.h>
#include <ServoESP32.h> //Use the Servo librarey for generating PWM

Servo throttle; //name the servo object, here ESC
Servo yaw;
Servo pitch;
Servo roll;

const int led = 2;

const int ref_throttle_pin = 26;
const int ref_yaw_pin = 25;
const int ref_pitch_pin = 33;
const int ref_roll_pin = 32;
const int flight_mode = 19;

int controller_active;
int controller_follow;

// Reference values received from the server
int* ref_array; //array that will hold four joystick outputs. throttle, yaw, pitch, roll.
float ref_throttle,
      ref_yaw,
      ref_pitch,
      ref_roll;

void updateSignal() {
  if (controller_active) {
    if (controller_follow) {
      Serial.println("Following...");
    }
    else {
      Serial.print("throttle: "); Serial.print(ref_throttle);
      Serial.print(" yaw: "); Serial.print(ref_yaw);
      Serial.print(" pitch: "); Serial.print(ref_pitch);
      Serial.print(" roll: "); Serial.println(ref_roll);


      //write
      throttle.writeMicroseconds(ref_throttle);
      yaw.writeMicroseconds(ref_yaw);
      pitch.writeMicroseconds(ref_pitch);
      roll.writeMicroseconds(ref_roll);
    }

  }
  else {
    Serial.println("Disarmed. Shutting off motors..");
    throttle.writeMicroseconds(1000);
    yaw.writeMicroseconds(1000);
  }

}

void readJoystick() {
  ref_array = getControlReference();
  controller_active = ref_array[4];
  controller_follow = ref_array[5];
  ref_throttle = map(ref_array[0], 0, 200, 1050, 1950);
  ref_yaw = map(ref_array[1], -100, 100, 1050, 1950);
  ref_pitch = map(ref_array[2], -100, 100, 1050, 1950);
  ref_roll = map(ref_array[3], -100, 100, 1050, 1950);


}


void setup() {
  Serial.begin(115200);
  connectWifi();
  connectWebsocket();
  pinMode(led, OUTPUT);
  throttle.attach(ref_throttle_pin); //Generate PWM in pin 9 of Arduino
  yaw.attach(ref_yaw_pin);
  pitch.attach(ref_pitch_pin);
  roll.attach(ref_roll_pin);
}


void loop() {
  // reset reference value to 0 at the start of every loop. This is to prevent
  // motors from spinning when it loses connection with the server.
  ref_throttle = 0;
  ref_yaw = 0;
  ref_pitch = 0;
  ref_roll = 0;
  controller_active = false;
  if (WiFi.status() == WL_CONNECTED) {
    if (pollWebsocket()) {
      digitalWrite(led, HIGH);
      readJoystick();

      /*
                    Serial.print("throttle: "); Serial.print(ref_throttle);
                    Serial.print(" yaw: "); Serial.print(ref_yaw);
                    Serial.print(" pitch: "); Serial.print(ref_pitch);
                    Serial.print(" roll: "); Serial.print(ref_roll);
      */
      /*
        //write
        FLESC.writeMicroseconds(ref_throttle);
        FRESC.writeMicroseconds(ref_throttle);
        RLESC.writeMicroseconds(ref_throttle);
        RRESC.writeMicroseconds(ref_throttle);*/
    }
    else {
      digitalWrite(led, LOW);
      connectWebsocket();
    }
  }
  else {
    digitalWrite(led, LOW);
    connectWifi();
    connectWebsocket();
  }
  updateSignal();
}
