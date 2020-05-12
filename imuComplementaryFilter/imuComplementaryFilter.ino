
/* http://www.youtube.com/c/electronoobs

   This is an example where we configure te data of the MPU6050
   and read the gyro data and print it to the serial monitor

   Arduino pin    |   MPU6050
   5V             |   Vcc
   GND            |   GND
   A4             |   SDA
   A5             |   SCL
*/

//Includes
#include <Wire.h>

#define SCL 22 // pin 22 for I2C SCL for MPU6050
#define SDA 21 // pin 21 for I2C SDA for MPU6050

//Variables
float elapsedTime, timeCurr, timePrev;        //Variables for time control
int gyro_error = 0;                       //We use this variable to only calculate once the gyro data error
int16_t Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read
float Gyro_angle_x, Gyro_angle_y, Gyro_angle_z;  //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y, Gyro_raw_error_z; //Here we store the initial gyro data error


//Variables
int acc_error = 0;                       //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180 / 3.141592654;    //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read
float Acc_angle_x, Acc_angle_y;          //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

float Total_angle_x, Total_angle_y;

void init_imu() {
  Wire.begin(SDA, SCL);                           //begin the wire comunication
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission

  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro

  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
}

void calculate_gyro_error() {
  if (gyro_error == 0)
  {
    for (int i = 0; i < 200; i++)
    {
      Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68)
      Wire.write(0x43);                        //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true);         //We ask for just 6 registers

      Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
      Gyr_rawY = Wire.read() << 8 | Wire.read();
      Gyr_rawZ = Wire.read() << 8 | Wire.read();

      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX / 32.8);
      /*---Y---*/
      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY / 32.8);
      /*---Z---*/
      Gyro_raw_error_z = Gyro_raw_error_z + (Gyr_rawZ / 32.8);

      if (i == 199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x / 200;
        Gyro_raw_error_y = Gyro_raw_error_y / 200;
        Gyro_raw_error_z = Gyro_raw_error_z / 200;
        gyro_error = 1;
      }
    }
  }
}

void calculate_accel_error() {
  if (acc_error == 0)
  {
    for (int a = 0; a < 200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                       //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true);

      Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ; //each value needs two registres
      Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
      Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;

      /*---X---*/
      Acc_angle_error_x = Acc_angle_error_x + atan2(Acc_rawY, Acc_rawZ)*rad_to_deg;
      //Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));
      /*---Y---*/
      Acc_angle_error_y = Acc_angle_error_y + atan2(-1 * Acc_rawX, sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg;
      //Acc_angle_error_y = Acc_angle_error_y + ((atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg));

      if (a == 199)
      {
        Acc_angle_error_x = Acc_angle_error_x / 200;
        Acc_angle_error_y = Acc_angle_error_y / 200;
        acc_error = 1;
      }
    }
  }
}
int16_t* get_gyro_xyz() {
  //////////////////////////////////////Gyro read/////////////////////////////////////

  Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68)
  Wire.write(0x43);                        //First adress of the Gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);         //We ask 6 registers

  Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
  Gyr_rawY = Wire.read() << 8 | Wire.read();
  Gyr_rawZ = Wire.read() << 8 | Wire.read();
  /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
  /*---X---*/
  Gyr_rawX = (Gyr_rawX / 32.8) - Gyro_raw_error_x;
  /*---Y---*/
  Gyr_rawY = (Gyr_rawY / 32.8) - Gyro_raw_error_y;
  /*---Z---*/
  Gyr_rawZ = (Gyr_rawZ / 32.8) - Gyro_raw_error_z;

  int16_t gyro_xyz[3];
  gyro_xyz[0] = Gyr_rawX;
  gyro_xyz[1] = Gyr_rawY;
  gyro_xyz[2] = Gyr_rawZ;

  return gyro_xyz;
}

float* get_angle_xy() {

  Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68)
  Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);      //keep the transmission and next
  Wire.requestFrom(0x68, 6, true);  //We ask for next 6 registers starting withj the 3B

  Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0 ; //each value needs two registres
  Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
  Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;

  /*---X---*/
  Acc_angle_x = atan2(Acc_rawY, Acc_rawZ)*rad_to_deg - Acc_angle_error_x;
  //Acc_angle_x = (atan((Acc_rawY) / sqrt(pow((Acc_rawX), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_x;
  /*---Y---*/
  Acc_angle_y = atan2(-1 * Acc_rawX, sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg - Acc_angle_error_y;
  //Acc_angle_y = (atan(-1 * (Acc_rawX) / sqrt(pow((Acc_rawY), 2) + pow((Acc_rawZ), 2))) * rad_to_deg) - Acc_angle_error_y;


  timePrev = timeCurr;                        // the previous time is stored before the actual time read
  timeCurr = micros();                        // actual time read
  elapsedTime = (timeCurr - timePrev) / 1000000; //divide by 1000000 in order to obtain seconds
  /*---X---*/
  Gyro_angle_x = Gyro_angle_x  + Gyr_rawX * elapsedTime;
  /*---Y---*/
  Gyro_angle_y = Gyro_angle_y  + Gyr_rawY * elapsedTime;


  // Complementary Filter
  /*---X axis angle---*/
  Total_angle_x = 0.98 * (Total_angle_x + Gyro_angle_x) + 0.02 * Acc_angle_x;
  /*---Y axis angle---*/
  Total_angle_y = 0.98 * (Total_angle_y + Gyro_angle_y) + 0.02 * Acc_angle_y;
}


void setup() {
  init_imu();
  calculate_gyro_error();
  calculate_accel_error();

  Serial.begin(115200);                     //Remember to set this same baud rate to the serial monitor
  timeCurr = micros();                        //Start counting time in mucroseconds

}


void loop() {
  get_gyro_xyz();
  get_angle_xy();




  Serial.print(Gyr_rawX);
  Serial.print("   |   ");

Serial.println(Gyr_rawY);


  /*Serial.print("GyroX angle: ");
    Serial.print(Gyro_angle_x);
    Serial.print("   |   ");
    Serial.print("GyroY angle: ");
    Serial.println(Gyro_angle_y);
*/


  /*
    Serial.print("GyroX raw: ");
    Serial.print(Gyr_rawX);
    Serial.print("   |   ");
    Serial.print("GyroY raw: ");
    Serial.print(Gyr_rawY);
    Serial.print("   |   ");
    Serial.print("GyroZ raw: ");
    Serial.println(Gyr_rawZ);
  */
  /*
    Serial.print("GyroX angle: ");
    Serial.print(Gyro_angle_x);
    Serial.print("   |   ");
    Serial.print("GyroY angle: ");
    Serial.println(Gyro_angle_y);
  */
}
