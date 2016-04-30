/*
  BlinkM.cpp - Library for controlling a BlinkM over i2c
  Created by Tim Koster, August 21 2013.
*/
#include "Marlin.h"
#ifdef BLINKM

#include "BlinkM.h"

void SendColors(unsigned char red, unsigned char grn, unsigned char blu, int led)
{
	char buf[256];
	char nr[5];

	snprintf(nr, sizeof(nr), "%d", led);

	//printf("Set LED nr.%s to R%d G%d B%d\n\r",nr,red,grn,blu);

	sprintf(buf, "red_%s", nr);
	setPwmFrequency(buf, red);

	sprintf(buf, "green_%s", nr);
	setPwmFrequency(buf, grn);

	sprintf(buf, "blue_%s", nr);
	setPwmFrequency(buf, blu);
}

#endif //BLINKM

