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
#include "pc_terminal/protocol.h"

/*-----------------------------------------------------------------------------------------
* convert_to_rpm() -	function to convert the raw values of lift, roll, pitch and yaw to
* 						corresponding rotor rpm values.
*
* Author: Himanshu Shah
* Date : 13/05/18
*------------------------------------------------------------------------------------------
*/
void convert_to_rpm(uint8_t lift, int8_t roll, int8_t pitch, int8_t yaw){
	int16_t rotor[4];
	rotor[0] = (int16_t)((int16_t)(4*lift) + 2*pitch - yaw);
	rotor[1] = (int16_t)((int16_t)(4*lift) - 2*roll + yaw);
	rotor[2] = (int16_t)((int16_t)(4*lift) - 2*pitch - yaw);
	rotor[3] = (int16_t)((int16_t)(4*lift) + 2*roll + yaw);

	for(uint8_t i=0; i<4; i++){
		if(rotor[i] < 0){
			rotor[i] = 0;   			
		}/*
		else{
			rotor[i] = (int)sqrt(rotor[i]);
		}*/
	}

	for(uint8_t i=0; i<4; i++){
		if(lift > 10 && rotor[i] < 200){
			if(rotor[i] > 0){
				rotor[i] = 200;
			}else{
				rotor[i] = 0;							// do not start rotors if lift is not provided.
			}
		}
		if(rotor[i] >= 500){
			rotor[i] = 500;
		}
	}

	for(uint8_t i=0; i<4; i++){
		ae[i] = rotor[i];
		//printf(" ae[%d]=%d", i, ae[i]);
	}
	//printf("\n");
}


void update_motors(void)
{					
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

void run_filters() // 100Hz (or different if DMP speed is changed)
{
	// fancy stuff here
	// filters n shiit
}

/*-----------------------------------------------------------------------------------------
* run_control() -	function that runs the quadcopter state machine.
*
* Author: Mark Röling
* Date : 17/05/2018
*------------------------------------------------------------------------------------------
*/
void run_control() // 250Hz
{
	static uint16_t panic_counter = 0;

	switch(QuadState){
		case SAFE:
			ae[0] = 0;
			ae[1] = 0;
			ae[2] = 0;
			ae[3] = 0;
			break;
		case PANIC:
			printf("Initiate PANIC mode.\n");
			//initiate PANIC mode
			panic_counter = 1005;
			QuadState = PANIC_COUNTDOWN;
		case PANIC_COUNTDOWN:
			//Slow down motors in ~3 seconds to 0
			if(--panic_counter>0){
				if(ae[0]>0) ae[0]--;
				if(ae[1]>0) ae[1]--;
				if(ae[2]>0) ae[2]--;
				if(ae[3]>0) ae[3]--;
			}else{
				QuadState = SAFE;
				printf("Initiate SAFE mode.\n");
			}
			break;
		case MANUAL:
			//Map the received values directly to the motors.
			printf("Lift: %d, Roll: %d, Pitch: %d, Yaw: %d\n", (uint8_t)LRPY[0], (int8_t)LRPY[1], (int8_t)LRPY[2], (int8_t)LRPY[3]);
			convert_to_rpm((uint8_t)LRPY[0], (int8_t)LRPY[1], (int8_t)LRPY[2], (int8_t)LRPY[3]);
			break;
		case CALIBRATION:
			QuadState = PANIC;
			break;
		case YAWCONTROL:
			QuadState = PANIC;
			break;
		case FULLCONTROL:
			QuadState = PANIC;
			break;
		case RAW:
			QuadState = PANIC;
			break;
		case HEIGHT:
			QuadState = PANIC;
			break;
		case WIRELESS:
			QuadState = PANIC;
			break;
		case DUMPLOGS:
			QuadState = PANIC;
			break;
		default:
			QuadState = PANIC;
			break;


	}
	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}
