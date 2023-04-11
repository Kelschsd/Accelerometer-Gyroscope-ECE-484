// Basic demo for accelerometer readings from Adafruit MPU6050
// Modified by Samuel Kelsch to test custom library simple6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "libs/simple6050.h"

Adafruit_MPU6050 mpu;

void setup(void) {

  simple6050_setup();

  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  //mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  /*Wire.beginTransmission(MPU);
  Wire.write(0x1A);
  Wire.write(0x06);
  Wire.endTransmission(true);*/
  //mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //Serial.println("");
  //delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);
  simple6050_read();
  /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(acc_x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(acc_y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(acc_z);
  Serial.print(", ");
  Serial.print("GyroAngleX:");
  Serial.print(gyroAngleX);
  Serial.print(",");
  Serial.print("GyroAngleY:");
  Serial.print(gyroAngleY);
  Serial.print(",");
  Serial.print("GyroAngleZ:");
  Serial.print(gyroAngleZ);
  Serial.println("");

  delay(10);
}