/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input =  0-1000 : 125-250 us (OneShot125)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"

void update_motors(void)
{					
	if (ae[0] >= 400) {
		motor[0] = 400;
	}
	else {
		motor[0] = ae[0];
	}

	if (ae[1] >= 400) {
		motor[1] = 400;
	}
	else {
		motor[1] = ae[1];
	}

	if (ae[2] >= 400) {
		motor[2] = 400;
	}
	else {
		motor[2] = ae[2];
	}

	if (ae[3] >= 400) {
		motor[3] = 400;
	}
	else {
		motor[3] = ae[3];
	}
}

void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}
