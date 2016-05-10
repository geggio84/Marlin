/*
  BlinkM.cpp - Library for controlling a BlinkM over i2c
  Created by Tim Koster, August 21 2013.
*/
#include "Marlin.h"
#ifdef BLINKM

#include "BlinkM.h"

static unsigned char tmp_red[MAX_NR_LEDS], tmp_grn[MAX_NR_LEDS], tmp_blu[MAX_NR_LEDS];

void SendColors(unsigned char red, unsigned char grn, unsigned char blu, int led)
{
	char buf[256];
	char nr[5];

	if(led > (MAX_NR_LEDS - 1)) {
		printf("ERROR : Trying to set led nr.%d that is > MAX_NR_LEDS (%d)\n", led, (MAX_NR_LEDS - 1));
		return;
	}
	snprintf(nr, sizeof(nr), "%d", led);

	//printf("Set LED nr.%s to R%d G%d B%d\n\r",nr,red,grn,blu);

	if (red != tmp_red[led]) {
		tmp_red[led] = red;
		sprintf(buf, "red_%s", nr);
		setPwmFrequency(buf, red);
	}

	if (grn != tmp_grn[led]) {
		tmp_grn[led] = grn;
		sprintf(buf, "green_%s", nr);
		setPwmFrequency(buf, grn);
	}

	if (blu != tmp_blu[led]) {
		tmp_blu[led] = blu;
		sprintf(buf, "blue_%s", nr);
		setPwmFrequency(buf, blu);
	}
}

#endif //BLINKM

