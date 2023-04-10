# Simple6050
  Midterm Project for ECE 484 Embedded systems where I interfaced with the MPU6050 accelerometer and gyroscope sensor.
This sensor is a low cost option for measuring orientation and acceleration in three dimentsons. The goal of this 
project was to create a library to allow a user to implement this sensor is a project very easily and with as few 
function calls and initialization steps as possible. The goal of the library was to setup default values for the 
sensor's configurations, but also to clearly explain other options if the user wants to change them.
This project was also required to be written in C, and be compiled using AVR with the intended microprocessor being
an Atmega328p, more than likely on an Arduino Uno.

# Implementing
  This library contains code to interface with the MPU6050 and additional libraries for establishing the I2C communication
with the sensor. To use this library, add the simple6050 to your project and include "simple6050.h" in your main program.
Initialization:
simple6050_setup();
Call from your main setup to initialize the 6050. There are no arguments for this, but if you wish to
change default settings, edit simple6050.h. 
Read data:
simple6050_read();
Call this whenever you want to take a reading, there are no arguments for this function. The values are stored into float variables:
acc_x         - Acceleration values in g's in x direction
acc_y         - Acceleration values in g's in y direction
acc_z         - Acceleration values in g's in z direction
gyro_x        - Acceleration values in g's on x axis
gyro_y        - Acceleration values in g's on y axis
gyro_z        - Acceleration values in g's on z axis
gyroAngleX    - Current angle of x axis, same as roll
gyroAngleY    - Current angle of y axis, same as pitch
gyroAnglez    - Same as yaw, cannot be accurately measured without additional hardware and will experience drift.

# Problems
Currently, cannot verify readings are correctly transmitted over I2C bus. Need to remove extraneous libraries from other attempts. 
