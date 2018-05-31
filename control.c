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
void convert_to_rpm(uint32_t lift, int32_t roll, int32_t pitch, int32_t yaw){
	int32_t rotor[4];
	rotor[0] = ((uint32_t)(5*lift) + (10*pitch) - (8*yaw));
	rotor[1] = ((uint32_t)(5*lift) - (10*roll) + (8*yaw));
	rotor[2] = ((uint32_t)(5*lift) - (10*pitch) - (8*yaw));
	rotor[3] = ((uint32_t)(5*lift) + (10*roll) + (8*yaw));
	//printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", rotor[0], rotor[1],rotor[2],rotor[3]);

	for(uint8_t i=0; i<4; i++){
		if(rotor[i] < 0){
			rotor[i] = 0;
		}
		else{
			rotor[i] = (uint16_t)sqrt(rotor[i]);
		}	
	}

	for(uint8_t i=0; i<4; i++){
		rotor[i] += k_LRPY[i]; 
	}

	for(uint8_t i=0; i<4; i++){
		if(lift<10){
			rotor[i]=0;
		}
		
		if(lift > 10 && rotor[i] < 200){
			rotor[i] = 200;
		}
		if(rotor[i] >= 700){
			rotor[i] = 700;
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

	int8_t kp = 10;
	int32_t yaw_error, adjusted_yaw;

	//YAW16=LRPY[3]<<8;
	for(uint8_t i =0; i<4; i++){
		LRPY16[i]=LRPY[i]<<8;
	}

	if(LRPY[0] > 10 || LRPY[0] < -10){
		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
		}
		
		kp += k_LRPY[4];
		if(kp < 1){
			kp = 1;
		}
		yaw_error = LRPY16[3] + k_LRPY[3] + sr;								//take keyboard offset into account
		adjusted_yaw = kp * yaw_error;

		convert_to_rpm((uint16_t)LRPY16[0], LRPY16[1], LRPY16[2], adjusted_yaw);
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d, sr:%d\n", ae[0],ae[1],ae[2],ae[3], sr);
		//printf("kp:%d\n", kp);	
	}
	else{
		for(uint8_t i=0; i<4; i++){
			ae[i]=0;
		}
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0], ae[1],ae[2],ae[3]);
	}
}

/*
*-----------------------------------------------------------------------------------------
* full_control() -	
* 					
*
* Author: Satish Singh
* Date : 23/05/18
*------------------------------------------------------------------------------------------
*/



void full_control(){

	int16_t LRPY16[4];
	int16_t roll_error, pitch_error, yaw_error, adjusted_roll, adjusted_pitch, adjusted_yaw;
	int8_t kp = 1,kp1 = 1,kp2 = 1;

	for(int8_t i=0; i<4; i++){
		LRPY16[i] = LRPY[i]<<8;
	}

	if(LRPY[0] > 10 || LRPY[0] < -10){
		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
		}
		
		kp += k_LRPY[4];
		if(kp < 1){
			kp = 1;
			k_LRPY[4]=0;
		}
		kp1 += k_LRPY[5];
		if(kp1 < 1){
			kp1 = 1;
			k_LRPY[5]=0;
		}
		kp2 += k_LRPY[6];
		if(kp2 < 1 ){
			kp2 = 1;
			k_LRPY[6]=0;
		}
		roll_error = LRPY16[1] - (k_LRPY[1]*4) - phi;
		pitch_error = LRPY16[2] - (k_LRPY[2]*4) - theta;
		yaw_error = LRPY16[3] + (k_LRPY[3]*4) + sr;								//take keyboard offset into account
		adjusted_pitch = (kp1 * pitch_error)/4 + (kp2 * sq)/2;
		adjusted_roll = (kp1 * roll_error)/4 - (kp2 * sp)/2;
		adjusted_yaw = kp * yaw_error;

		convert_to_rpm((uint16_t)LRPY16[0], adjusted_roll, adjusted_pitch, adjusted_yaw);
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0], ae[1],ae[2],ae[3]);
		//printf("kp:%d,kp1:%d,kp2:%d\n", kp, kp1,kp2);	
	}
	else{
		for(uint8_t i=0; i<4; i++){
			ae[i]=0;
		}
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0], ae[1],ae[2],ae[3]);
	}
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
#define NON_ZERO_DEBUG	 1
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
			for(uint8_t i=0; i<4; i++){
				LRPY16[i]=LRPY[i]<<8;
			}
			convert_to_rpm((uint16_t)LRPY16[0],LRPY16[1], LRPY16[2], LRPY16[3]);
			printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0],ae[1],ae[2],ae[3]);
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
			full_control();
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
			printf("INFO: Log dump started.\n");
			// Iterate starting from last address that contains data
			uint32_t i = write_addr - LOG_ENTRY_SIZE_BYTES;
			// Normal readout
			while(i >= 0){
				if(!read_log_entry(i)){
					printf("ERROR: Log dump aborted early.\n");
					flash_overflow = false;
					break;
				} else {
					i -= LOG_ENTRY_SIZE_BYTES;
				}
			}
			// Overflow readout
			if(flash_overflow){
				// Read back from end of flash to last non-erased address
				i = addr_before_overflow - LOG_ENTRY_SIZE_BYTES;
				uint32_t lower_addr_limit = (curr_flash_block + 1) * 4000;
				// Read down to lower address limit
				while(i > lower_addr_limit){
					if(!read_log_entry(i)){
						printf("ERROR: Log dump aborted early.\n");
						break;
					} else {
						i -= LOG_ENTRY_SIZE_BYTES;
					}
				}			
			}
			printf("INFO: Log dump complete.\n");
			QuadState = SAFE;
			break;
		case SETNEWMODE:
			// Do nothing if we want the same mode again.
			if(PreviousMode == ModeToSet){
				QuadState = PreviousMode;
				break;
			}

			// Only go to DUMPLOGS and CALIBRATION from SAFE modes.
			if((ModeToSet == DUMPLOGS || ModeToSet == CALIBRATION )){
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
