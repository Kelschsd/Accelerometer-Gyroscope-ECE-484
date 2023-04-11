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
* Written by Samuel Kelsch (kelschsd@miamioh.edu)
*
* Code referenced and included from:
* (1) https://github.com/adafruit/Adafruit_MPU6050
* (2) https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
*/

#include "i2c_master.c"  // Used for I2C communications
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <float.h>
#include <util/delay.h>
#include <avr/io.h>
#include "uart.c"

const int MPU = 0x68;  // I2C address for MPU6050
float acc_x, acc_y, acc_z;                                          // Acceleration values in g's in each dimention
float gyro_x, gyro_y, gyro_z;                                       // Acceleration values in g's on each axis
uint8_t sen_acc, sen_gyro, cutoff;
uint8_t data_in[6];
float lsb_sen_acc, lsb_sen_gyro;
float accAngleX, accAngleY;
float gyroAngleX, gyroAngleY, gyroAngleZ;                           // Current angle of each axis
float roll, pitch;                                                  // roll is over x-axis, pitch is over y-axis
float elapsedTime, currentTime, previousTime;
time_t timer;
float acc_x_S;
char buf[10];		// Used to print out values					

/*
*	ChatGPT Generated
*/
char *ftoa(float f, char *buf, int precision) {
    char *ptr = buf;
    sprintf(buf, "%d.", (int)f);
    while (*ptr != '.') ptr++;
    for (int i = 0; i < precision; i++) {
        f = (f - (int)f) * 10;
        *++ptr = (int)f + '0';
    }
    *++ptr = '\0';
    return buf;
}


/*
* Sets initial values of parameters for 6050
* Other options can be commented out as needed
*/
void simple6050_setup()
{
	uart_init(9600); // bps
	cli_reset();
	printf("\n");
	printf("Initializing 6050 \n");

  /*
  * Modified from (2)
  */
  i2c_master_init(I2C_SCL_FREQUENCY_400);       // Initialize comunication block
  i2c_master_sendByte(0x6B, 0x00);
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  // Make range smaller for more sensitive readings
  /* =============Choose One================*/
  sen_acc = 0x00;                    //Set the register bits as 00000000 (+/- 2g full scale range)
  //sen_acc = 0x08;                  //Set the register bits as 00001000 (+/- 4g full scale range)
  //sen_acc = 0x10;                  //Set the register bits as 00010000 (+/- 8g full scale range)
  //sen_acc = 0x18;                  //Set the register bits as 00011000 (+/- 16g full scale range)
  /* ========================================*/
  i2c_master_sendByte(0x1C, sen_acc);	//Talk to the ACCEL_CONFIG register (1C hex)

  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  // Make smaller range for more sensitive readings
  /* ==============Choose One================*/
  sen_gyro = 0x00;                     // Set the register bits as 00000000 (250deg/s full scale)
  //sen_gyro = 0x08;                   // Set the register bits as 00001000 (500deg/s full scale)
  //sen_gyro = 0x10;                   // Set the register bits as 00010000 (1000deg/s full scale)
  //sen_gyro = 0x18;                   // Set the register bits as 00011000 (2000deg/s full scale)
  /* ========================================*/
   i2c_master_sendByte(0x1B, sen_gyro);		// Talk to the GYRO_CONFIG register (1B hex)

  /*
  * Configure Digital Low Pass Filter
  * Can increase filter cutoff to eliminate high frequency noise
  * Configures cutoff for both accelerometer and gyroscope
  */
   /* =============Choose One================*/
                                          // Set the register to # (cutoff = <accelerometer/gyro>)
  cutoff = (0x00);                     // Set the register to 0 (cutoff = 260 Hz / 8 kHz ) - No Filter
  //cutoff = (0x01);                     // Set the register to 1 (cutoff = 184 Hz / 1 kHz )
  //cutoff = (0x02);                     // Set the register to 2 (cutoff =  94 Hz / 1 kHz )
  //cutoff = (0x03);                     // Set the register to 3 (cutoff =  44 Hz / 1 kHz )
  //cutoff = (0x04);                     // Set the register to 4 (cutoff =  21 Hz / 1 kHz )
  //cutoff = (0x05);                     // Set the register to 5 (cutoff =  10 Hz / 1 kHz )
  //cutoff = (0x06);                     // Set the register to 6 (cutoff =   5 Hz / 1 kHz ) - Strong Filter
   /* =======================================*/
   i2c_master_sendByte(0x1A, cutoff);		// Talk to the DLPF_CFG register
   

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
	i2c_master_receive(MPU, &data_in[0], 1);
	i2c_master_receive(MPU, &data_in[1], 1);
	i2c_master_receive(MPU, &data_in[2], 1);
	// Code blocks modified from (2)
	// Must divide by sensitivity values
	acc_x = data_in[0] / lsb_sen_acc; // X-axis value
	acc_y = data_in[1] / lsb_sen_acc; // Y-axis value
	acc_z = data_in[2] / lsb_sen_acc; // Z-axis value

	i2c_master_receive(MPU, &data_in[4], 1);
	i2c_master_receive(MPU, &data_in[5], 1);
	i2c_master_receive(MPU, &data_in[6], 1);
	gyro_x = data_in[4] / lsb_sen_gyro;
	gyro_y = data_in[5] / lsb_sen_gyro;
	gyro_z = data_in[6] / lsb_sen_gyro;

	previousTime = currentTime;     	 // Previous time is stored before the actual time read
	currentTime = timer;           		 // Current time actual time read
	elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

	// Accelerometer offset. Values obtained through inspection of plot.
	acc_x = acc_x - 0.1;
	acc_y = acc_y + 0.03;

	// Calculating Roll and Pitch from the accelerometer data from (2)
	accAngleX = (atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / M_PI);
	accAngleY = (atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / M_PI) ; 

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
	uart_init(9600); // bps
	cli_reset();
	//printf("TESTING VALUES\n");
	simple6050_read();
	/* Print out the values */
	printf("AccelX:");
	ftoa(acc_x, buf, 2);
    printf("%s", buf);
	printf(",");
	printf("AccelY:");
	ftoa(acc_y, buf, 2);
    printf("%s", buf);
	printf(",");
	printf("AccelZ:");
	ftoa(acc_z, buf, 2);
    printf("%s\n", buf);
	printf("GyroAngleX:");
	ftoa(gyroAngleX, buf, 2);
    printf("%s", buf);
	printf(",");
	printf("GyroAngleY:");
	ftoa(gyroAngleY, buf, 2);
    printf("%s", buf);
	printf(",");
	printf("GyroAngleZ:");
	ftoa(gyroAngleZ, buf, 2);
    printf("%s\n", buf);
	//printf("roll:");
	//printf("%f",roll);
	//printf(",");
	//printf("pitch:");
	//printf("%f \n",pitch);
	_delay_ms(30);
}






