#include "wifi_websocket.h"
#include "imu.h"
#include <WiFi.h>
#include <ServoESP32.h> //Use the Servo librarey for generating PWM

Servo FLESC; //name the servo object, here ESC
Servo FRESC;
Servo RLESC;
Servo RRESC;

const int flEscPin = 25;
const int frEscPin = 26;
const int rlEscPin = 32;
const int rrEscPin = 33;

// current yaw pitch roll values in degrees
float* curr_ypr;
// current rotational rate in degrees/second
float gx, gyro_roll, gz;

int controller_active;

// Reference values received from the server
int* ref_array; //array that will hold four joystick outputs. throttle, yaw, pitch, roll.
float ref_throttle,
      ref_yaw,
      ref_pitch,
      ref_roll;

//PID constants
const float kp_roll = 1.9;
const float kd_roll = 15;
const float ki_roll = 0.015;

// derivative and integral errors
float prev_err_roll = 0,
      eint_roll = 0;

void updateMotor() {
  if (controller_active) {
    float u_throttle = ref_throttle;

    // do calculation **ROLL**
    float err_roll = 0;
    float edot_roll = 0;
    float u_roll = 0;
    float curr_roll = curr_ypr[2] * -180 / M_PI;

    err_roll = ref_roll - gyro_roll;
    edot_roll = err_roll - prev_err_roll;
    eint_roll = eint_roll + err_roll;
  
    u_roll = (kp_roll * err_roll) + (kd_roll * edot_roll) + (ki_roll * eint_roll);

    int fl = u_throttle + u_roll;
    int fr = u_throttle - u_roll;
    int rl = u_throttle + u_roll;
    int rr = u_throttle - u_roll;

    if ((fl>1300) || (fr>1300) || (rl>1300) || (rr>1300)){
      eint_roll = 0;
    }
    
    Serial.print("fl "); Serial.print(fl);
    Serial.print("fr "); Serial.print(fr);
    Serial.print("rl "); Serial.print(rl);
    Serial.print("rr "); Serial.println(rr);
    /*
      // update new output signal
      int fl = u_throttle + u_yaw + u_pitch - u_roll;
      int fr = u_throttle - u_yaw + u_pitch + u_roll;
      int rl = u_throttle - u_yaw - u_pitch - u_roll;
      int rr = u_throttle + u_yaw - u_pitch + u_roll;
    */

    //write
    FLESC.writeMicroseconds(fl);
    FRESC.writeMicroseconds(fr);
    RLESC.writeMicroseconds(rl);
    RRESC.writeMicroseconds(rr);

    prev_err_roll = err_roll;
  }
  else {
    Serial.println("Disamred. Shutting off motors..");
    FLESC.writeMicroseconds(0);
    FRESC.writeMicroseconds(0);
    RLESC.writeMicroseconds(0);
    RRESC.writeMicroseconds(0);
    eint_roll = 0;
  }

}

void readJoystick() {
  ref_array = getControlReference();

  ref_throttle = map(ref_array[0], 0, 200, 950, 1200);
  ref_yaw = map(ref_array[1], -100, 100, -10, 10);
  ref_pitch = map(ref_array[2], -100, 100, -10, 10);
  ref_roll = map(ref_array[3], -100, 100, -10, 10);
  controller_active = ref_array[4];
}


void setup() {
  Serial.begin(115200);
  connectWifi();
  connectWebsocket();
  initIMU();
  FLESC.attach(flEscPin); //Generate PWM in pin 9 of Arduino
  FRESC.attach(frEscPin);
  RLESC.attach(rlEscPin);
  RRESC.attach(rrEscPin);
}


void loop() {
  ref_throttle = 0;
  ref_yaw = 0;
  ref_pitch = 0;
  ref_roll = 0;
  controller_active = false;
  if (WiFi.status() == WL_CONNECTED) {
    if (pollWebsocket()) {
      curr_ypr = getYPR();
      getRotation(&gx, &gyro_roll, &gz);
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
      connectWebsocket();
    }
  }
  else {
    connectWifi();
    connectWebsocket();
  }
  updateMotor();
}
