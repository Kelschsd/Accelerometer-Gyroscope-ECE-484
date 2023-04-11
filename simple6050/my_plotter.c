// Basic demo for accelerometer readings from Adafruit MPU6050
// Modified by Samuel Kelsch to test custom library simple6050

#include "libs/simple6050.c"
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

void setup(void) {
  simple6050_setup();
  
}

int main() {
	setup();
	DDRB |= _BV(DDB5);
	while(1){
		if (gyroAngleX > 0){
			PORTB |= _BV(PORTB5);
			_delay_ms(100);
		}
		PORTB &= ~_BV(PORTB5);
		_delay_ms(100);
		//simple6050_test();
		/* Get new sensor events with the readings */
		simple6050_test();
		/* Print out the values */
		/* printf("AccelX:");
		printf(acc_x);
		printf(",");
		printf("AccelY:");
		printf(acc_y);
		printf(",");
		printf("AccelZ:");
		printf(acc_z);
		printf(", ");
		printf("GyroAngleX:");
		printf(gyroAngleX);
		printf(",");
		printf("GyroAngleY:");
		printf(gyroAngleY);
		printf(",");
		printf("GyroAngleZ:");
		printf(gyroAngleZ);
		printfln(""); 
		printf("Accel X: %5.2f\n", acc_x);
		printf("Accel Y: %5.2f\n", acc_y);
		printf("Accel Z: %5.2f\n", acc_z); */
		_delay_ms(10);
	}
	return 0;
}