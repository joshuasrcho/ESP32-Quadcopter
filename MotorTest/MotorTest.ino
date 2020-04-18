#include <ServoESP32.h> //Use the Servo librarey for generating PWM
Servo FLESC; //name the servo object, here ESC
Servo FRESC;
Servo RLESC;
Servo RRESC;

const int flEscPin = 25;
const int frEscPin = 26;
const int rlEscPin = 32;
const int rrEscPin = 33;
const int potPin = 4;

void setup()
{
  Serial.begin(115200);
  FLESC.attach(flEscPin); //Generate PWM in pin 9 of Arduino
  FRESC.attach(frEscPin);
  RLESC.attach(rlEscPin);
  RRESC.attach(rrEscPin);
}

void loop()
{
  int servoPosition = map(analogRead(potPin), 0, 4096, 40, 150);
  FLESC.write(servoPosition);
  FRESC.write(servoPosition);
  RLESC.write(servoPosition);
  RRESC.write(servoPosition);
  Serial.println(servoPosition);
}
