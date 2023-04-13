// Basic demo for accelerometer readings from Adafruit MPU6050
// Modified by Samuel Kelsch to test custom library simple6050

#include "libs/simple6050.c"
#include <util/delay.h>

void setup(void) {
  simple6050_setup();
}

int main() {
	setup();
	while(1){
		/* Get new sensor events with the readings */
		//simple6050_read();
		/* Get new sensor events and print the readings */
		simple6050_test();
		_delay_ms(10);
	}
	return 0;
}