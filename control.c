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
#include <stdlib.h>


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
	rotor[0] = (int16_t)((int16_t)2*(lift + k_LRPY[0]) + 2*(pitch + k_LRPY[2]) - (yaw + k_LRPY[3]));
	rotor[1] = (int16_t)((int16_t)2*(lift + k_LRPY[0]) - 2*(roll + k_LRPY[1]) + (yaw + k_LRPY[3]));
	rotor[2] = (int16_t)((int16_t)2*(lift + k_LRPY[0]) - 2*(pitch + k_LRPY[2]) - (yaw + k_LRPY[3]));
	rotor[3] = (int16_t)((int16_t)2*(lift + k_LRPY[0]) + 2*(roll + k_LRPY[1]) + (yaw + k_LRPY[3]));

	for(uint8_t i=0; i<4; i++){
		if(rotor[i] < 0){
			rotor[i] = 0;   			
		}
		/*
		else{
			rotor[i] = (int)sqrt(rotor[i]);
		}*/
	} 

	for(uint8_t i=0; i<4; i++){
		if(lift<10){rotor[i]=0;}
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


/*-----------------------------------------------------------------------------------------
* yaw_control() -	
* 					
*
* Author: Satish Singh
* Date : 23/05/18
*------------------------------------------------------------------------------------------
*/



void yaw_control(){

	//unsigned int range = 65535;
	//unsigned int new_range = 255;
	//unsigned int max = 32767;
	//int min = -32768;
	//int new_min = -128;
	//uint8_t kp;
	//uint8_t yaw_error;

	if(LRPY[0] > 10 || LRPY[0] < -10){
		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
		}
		/*sr=(int8_t)(((sr - min) * new_range) / range) + new_min;	
		if(k_LRPY[4] == 0){
			kp=1;
		}
		else{
			kp=k_LRPY[4];
		}
		yaw_error = LRPY[3] + k_LRPY[3] - sr;								//take keyboard offset into account
		LRPY[3]= kp * yaw_error;
		*/
		convert_to_rpm((uint8_t)LRPY[0], (int8_t)LRPY[1], (int8_t)LRPY[2], (int8_t)LRPY[3]);
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d, sr:%d\n", ae[0],ae[1],ae[2],ae[3], sr);
	}
	/*else{
		LRPY[0]=LRPY[1]=LRPY[2]=LRPY[3]=0;
		convert_to_rpm((uint8_t)LRPY[0], (int8_t)LRPY[1], (int8_t)LRPY[2], (int8_t)LRPY[3]);
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0],ae[1],ae[2],ae[3]);
	}*/
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
* near_zero() -	function that checks whether requested inputs are near-zero to
*				escape safe mode
*
* Author: Mark Röling
* Date : 21/05/2018
*------------------------------------------------------------------------------------------
*/
#define NON_ZERO_DEBUG	 0
#define LIFT_THRESSHOLD  2
#define ROLL_THRESSHOLD  5
#define PITCH_THRESSHOLD 5
#define YAW_THRESSHOLD   5
bool near_zero(void){
	if((uint8_t)LRPY[0] < LIFT_THRESSHOLD){ // lift
		if((int8_t)LRPY[1] < ROLL_THRESSHOLD && (int8_t)LRPY[1] > -ROLL_THRESSHOLD){ // roll
			if((int8_t)LRPY[2] < PITCH_THRESSHOLD && (int8_t)LRPY[2] > -PITCH_THRESSHOLD){ // pitch
				if((int8_t)LRPY[3] < YAW_THRESSHOLD && (int8_t)LRPY[3] > -YAW_THRESSHOLD){ // yaw
					return true;
				}else{
					#if NON_ZERO_DEBUG == 1
					printf("Yaw non-zero: %d\n", (uint8_t)LRPY[3]);
					#endif
				}
			}else{
				#if NON_ZERO_DEBUG == 1
				printf("Pitch non-zero: %d\n", (uint8_t)LRPY[2]);
				#endif
			}
		}else{
			#if NON_ZERO_DEBUG == 1
			printf("Roll non-zero: %d\n", (int8_t)LRPY[1]);
			#endif
		}
	}else{
		#if NON_ZERO_DEBUG == 1
		printf("Lift non-zero: %d\n", (uint8_t)LRPY[0]);
		#endif
	}
	return false;
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
	uint32_t cur_time = 0;

	switch(QuadState){
		case SAFE:
			//printf("S\n");
			ae[0] = 0;
			ae[1] = 0;
			ae[2] = 0;
			ae[3] = 0;
			if(!near_zero()){ //inputs are not near 0
				printf("Safe mode, non-zero.\n");
				QuadState = SAFE_NONZERO;
			}else{
				break;
			}
		case SAFE_NONZERO:
			//printf("SN\n");
			ae[0] = 0;
			ae[1] = 0;
			ae[2] = 0;
			ae[3] = 0;
			//move to state safe if inputs are near 0
			if(near_zero()){ //inputs are within safe margin of 0
				printf("Safe mode, zero.\n");
				QuadState = SAFE;
			}
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
				QuadState = SAFE_NONZERO;
				printf("Initiate SAFE_NONZERO mode.\n");
			}
			break;
		case MANUAL:
			//Map the received values directly to the motors.
			//printf("Lift: %d, Roll: %d, Pitch: %d, Yaw: %d\n", (uint8_t)LRPY[0], (int8_t)LRPY[1], (int8_t)LRPY[2], (int8_t)LRPY[3]);
			convert_to_rpm((uint8_t)LRPY[0], (int8_t)LRPY[1], (int8_t)LRPY[2], (int8_t)LRPY[3]);
			//printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0],ae[1],ae[2],ae[3]);
			break;
		case CALIBRATION_ENTER:
			printf("Initiate CALIBRATION mode.\n");
			CalibrationStartTime = get_time_us();
			phi_o = phi;
			theta_o = theta;
			psi_o = psi;
			sp_o = sp;
			sq_o = sq;
			sr_o = sr;
			QuadState = CALIBRATION;
		case CALIBRATION:
			cur_time = get_time_us();
			if(cur_time < CalibrationStartTime + CALIBRATION_TIME_US){
				phi_o -= ((phi_o - phi) >> 2);
				theta_o -= ((theta_o - theta) >> 2);
				psi_o -= ((psi_o - psi) >> 2);
				sp_o -= ((sp_o - sp) >> 2);
				sq_o -= ((sq_o - sq) >> 2);
				sr_o -= ((sr_o - sr) >> 2);
			}else{
				printf("Calibration done. Offsets: phi=%d, theta=%d, psi=%d, sp=%d, sq=%d, sr=%d.\n", phi_o, theta_o, psi_o, sp_o, sq_o, sr_o);
				QuadState = PANIC;
			}
			break;
		case YAWCONTROL:
			yaw_control();
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
			printf("Dumping logs:\n");
			for(uint16_t i=0; i+LOG_ENTRY_SIZE_BYTES<FLASH_ADDR_LIMIT; i+=LOG_ENTRY_SIZE_BYTES){
				if(!read_log_entry(i)){
					break;
				}
			}
			printf("\nDone Dumping logs.\n");
			QuadState = PANIC;
			break;
		case SETNEWMODE:
			// Do nothing if we want the same mode again.
			if(PreviousMode == ModeToSet){
				QuadState = PreviousMode;
				break;
			}

			// Only go to DUMPLOGS and CALIBRATION from SAFE modes.
			if((ModeToSet == DUMPLOGS || ModeToSet == CALIBRATION)){
				if(PreviousMode == SAFE || PreviousMode == SAFE_NONZERO){
					QuadState = ModeToSet;
					if(ModeToSet == CALIBRATION){
						QuadState = CALIBRATION_ENTER;
					}
					break;
				}else{
					QuadState = PreviousMode;
					break;
				}
			}


			// Don't allow any mode changes from PANIC or SAFE_NONZERO.
			if(PreviousMode != PANIC && PreviousMode != PANIC_COUNTDOWN && PreviousMode != SAFE_NONZERO){
				if(ModeToSet == MANUAL){
					printf("Initiated MANUAL mode.\n");
				}else if(ModeToSet == PANIC){
					printf("Initiated PANIC mode.\n");
				}else{
					printf("Initiated 0x%02X mode.\n", ModeToSet);
				}
				QuadState = ModeToSet;
				break;
			}

			// Assume all is normal.
			QuadState = PreviousMode;
			break;
		default:
			QuadState = PANIC;
			break;


	}
	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}
