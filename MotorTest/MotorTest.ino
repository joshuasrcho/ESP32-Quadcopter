#include <ServoESP32.h> //Use the Servo librarey for generating PWM
Servo ESC; //name the servo object, here ESC

const int escPin = 5;
const int potPin = 4;

void setup()
{
  Serial.begin(115200);
  ESC.attach(escPin); //Generate PWM in pin 9 of Arduino
}

void loop()
{
  int servoPosition = map(analogRead(potPin), 0, 4096, 40, 150);
  ESC.write(servoPosition);
  Serial.println(servoPosition);
}
