/*
* Driver for MPU6050 Accelerometer and Gyroscope 
* using I2C
* Many features are not utilized in this implementation
* to allow users to quickly implement an accelerometer/gyro
* into their project.
*
* =========MPU6050 Wiring=========
*		+---------+---------+
*		| MPU6050 | Arduino |
*		+---------+---------+
*		| VCC     | 5 V     |
*		| GND     | GND     |
*		| SCL     | A5      |
*		| SDA     | A4      |
*		+---------+---------+
*
* Written by Samuel Kelsch kelschsd@miamioh.edu
*
* Code referenced and included from:
* (1) https://github.com/adafruit/Adafruit_MPU6050
* (2) https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
*/

#include <Wire.h>  // Used for I2C communications

const int MPU = 0x68;  // I2C address for MPU6050
float acc_x, acc_y, acc_z;                                          // Acceleration values in g's in each dimention
float gyro_x, gyro_y, gyro_z;                                       // Acceleration values in g's on each axis
uint8_t sen_acc, sen_gyro;
float lsb_sen_acc, lsb_sen_gyro;
float accAngleX, accAngleY;
float gyroAngleX, gyroAngleY, gyroAngleZ;                           // Current angle of each axis
float roll, pitch;                                                  // roll is over x-axis, pitch is over y-axis
float elapsedTime, currentTime, previousTime;

// void setup(void){
//   Serial.begin(115200);
//   simple6050_setup();
// }

/*
* Sets initial values of parameters for 6050
* Other options can be commented out as needed
*/
void simple6050_setup()
{
  /*
  * Modified from (2)
  */
  Wire.begin();                 // Initialize comunication block
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   // End the transmission block

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  // Make range smaller for more sensitive readings
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  //Talk to the ACCEL_CONFIG register (1C hex)
  
  /* =============Choose One================*/
  sen_acc = 0x00;                    //Set the register bits as 00000000 (+/- 2g full scale range)
  //sen_acc = 0x08;                  //Set the register bits as 00001000 (+/- 4g full scale range)
  //sen_acc = 0x10;                  //Set the register bits as 00010000 (+/- 8g full scale range)
  //sen_acc = 0x18;                  //Set the register bits as 00011000 (+/- 16g full scale range)
  /* ========================================*/
  Wire.write(sen_acc);
  Wire.endTransmission(true);

  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  // Make smaller range for more sensitive readings
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  // Talk to the GYRO_CONFIG register (1B hex)

  /* ==============Choose One================*/
  sen_gyro = 0x00;                     // Set the register bits as 00000000 (250deg/s full scale)
  //sen_gyro = 0x08;                   // Set the register bits as 00001000 (500deg/s full scale)
  //sen_gyro = 0x10;                   // Set the register bits as 00010000 (1000deg/s full scale)
  //sen_gyro = 0x18;                   // Set the register bits as 00011000 (2000deg/s full scale)
  /* ========================================*/
  Wire.write(sen_gyro);
  Wire.endTransmission(true);

  /*
  * Configure Digital Low Pass Filter
  * Can increase filter cutoff to eliminate high frequency noise
  * Configures cutoff for both accelerometer and gyroscope
  */
  Wire.beginTransmission(MPU);
  Wire.write(0x1A);  // Talk to the DLPF_CFG register

   /* =============Choose One================*/
                                          // Set the register to # (cutoff = <accelerometer/gyro>)
  Wire.write(0x00);                     // Set the register to 0 (cutoff = 260 Hz / 8 kHz ) - No Filter
  //Wire.write(0x01);                     // Set the register to 1 (cutoff = 184 Hz / 1 kHz )
  //Wire.write(0x02);                     // Set the register to 2 (cutoff =  94 Hz / 1 kHz )
  //Wire.write(0x03);                     // Set the register to 3 (cutoff =  44 Hz / 1 kHz )
  //Wire.write(0x04);                     // Set the register to 4 (cutoff =  21 Hz / 1 kHz )
  //Wire.write(0x05);                     // Set the register to 5 (cutoff =  10 Hz / 1 kHz )
  //Wire.write(0x06);                     // Set the register to 6 (cutoff =   5 Hz / 1 kHz ) - Strong Filter
   /* =======================================*/
  Wire.endTransmission(true);

  /*
  * Switch cases used to set sensitivity values
  * given in the data sheet
  */
  switch(sen_acc){
    case 0x00 :
      lsb_sen_acc = 16384.0;
      break;
    case 0x08 :
      lsb_sen_acc = 8192.0;
      break;
    case 0x10 :
      lsb_sen_acc = 4096.0;
      break;
    case 0x18 :
      lsb_sen_acc = 2048.0;
      break;
    default :
      lsb_sen_acc = 0;
    // Error, this should never be a case
  }

  switch(sen_gyro){
    case 0x00 :
      lsb_sen_gyro = 131.0;
      break;
    case 0x08 :
      lsb_sen_gyro = 65.5;
      break;
    case 0x10 :
      lsb_sen_gyro = 32.8;
      break;
    case 0x18 :
      lsb_sen_gyro = 16.4;
      break;
    default :
      lsb_sen_gyro = 0;
    // Error, this should never be a case
  }

}

/*
* Method reads data from 6050's registers and 
* stores it into varriables
*/
void simple6050_read()
{
  // Code blocks modified from (2)
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  // Must divide by sensitivity values
  acc_x = (Wire.read() << 8 | Wire.read()) / lsb_sen_acc; // X-axis value
  acc_y = (Wire.read() << 8 | Wire.read()) / lsb_sen_acc; // Y-axis value
  acc_z = (Wire.read() << 8 | Wire.read()) / lsb_sen_acc; // Z-axis value

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  gyro_x = (Wire.read() << 8 | Wire.read()) / lsb_sen_gyro;
  gyro_y = (Wire.read() << 8 | Wire.read()) / lsb_sen_gyro;
  gyro_z = (Wire.read() << 8 | Wire.read()) / lsb_sen_gyro;

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  // Accelerometer offset. Values obtained through inspection of plot.
  acc_x = acc_x - 0.1;
  acc_y = acc_y + 0.03;

  // Calculating Roll and Pitch from the accelerometer data from (2)
  accAngleX = (atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / PI);
  accAngleY = (atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / PI) ; 

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + gyro_x * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + gyro_y * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
  gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;

  roll = gyroAngleX;
  pitch = gyroAngleY;
  // Yaw is not accurate with this method, additional hardware would be needed
}

/*
* A simple test loop that prints the readings to the serial
* monitor. Values can also be viewed in the serial plotter.
*/
void simple6050_test()
{
  simple6050_read();
  //   /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(acc_x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(acc_y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(acc_z);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(gyro_x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(gyro_y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(gyro_z);
  Serial.print(",");
  Serial.print("roll:");
  Serial.print(roll);
  Serial.print(",");
  Serial.print("pitch:");
  Serial.println(pitch);

  

  delay(30);
}

// void loop(){
//   simple6050_test();  
// }






