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

bool butter_flag = true;
uint8_t height_flag =0;
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

/*----------------------------------------------------------------
 * float2fix -- convert float to fixed point 18+14 bits
 *----------------------------------------------------------------
 */
int     float2fix(double x)
{
	int	y;

	y = x * (1 << 14);
	return y;
}


/*----------------------------------------------------------------
 * fix2float -- convert fixed 18+14 bits to float
 *----------------------------------------------------------------
 */
double 	fix2float(int x)
{
	double	y;

	y = ((double) x) / (1 << 14);
	return y;
}


/*----------------------------------------------------------------
 * fixmul -- multiply fixed 18+14 bits to float
 *----------------------------------------------------------------
 */
double 	fixmul(int x1, int x2)
{
	int	y;

	y = x1 * x2; // Note: be sure this fits in 32 bits !!!!
	y = (y >> 14);
	return y;
}


/*
*-----------------------------------------------------------------------------------------
* butterworth_filter() -	
* 					
*
* Author: Satish Singh
* Date : 03/06/18
*------------------------------------------------------------------------------------------
*/

void butterworth_filter(){
	static uint32_t x[3] = {0,0,0}, y[3] = {0,0,0}, gain, b1, b2, a0, a1, a2;
	if(butter_flag){
		gain = float2fix(14.82463775);
		a0 = float2fix(1);
		a1 = float2fix(2);
		a2 = float2fix(1);
		b2 = float2fix(0.4128015981);
		b1 = float2fix(1.1429805025);
		y[0] = float2fix(y[0]);
		y[1] = float2fix(y[1]);
		for(uint8_t i=0; i<3; i++){
			x[i] = float2fix(x[i]);
		}
	}
	//if(LRPY[0] > 10 || LRPY[0] < -10){
		if (check_sensor_int_flag()) 
		{
			get_raw_sensor_data();
		}
		for(uint8_t i=2; i>0; i--){
			x[i] = x[i-1];
			y[i] = y[i-1];
		}
		x[0] = float2fix(sr)/gain;
		y[0] = (fixmul(a0,x[0]) + fixmul(a1,x[1]) + fixmul(a2,x[2]) - fixmul(-b2,y[2]) - fixmul(b1,y[1]));
		filtered_sr = fix2float(y[0]);
	//}
	//else{
	//	for(uint8_t i=0; i<4; i++){
	//		ae[i]=0;
	//	}
		printf("sr: %06d filetered: %06ld\n", sr, 10*filtered_sr);
	//}
}



/*
*-----------------------------------------------------------------------------------------
* kalman_filter() -	
* 					
*
* Author: Satish Singh
* Date : 03/06/18
*------------------------------------------------------------------------------------------
*/


//void kalman_filter(){
//
//	static int32_t p2phi, new_p_bias, new_q_bias, old_p_bias, old_q_bias, c1, q, c2, p, new_phi, old_phi, new_theta, old_theta, phi_error, theta_error; 
//	if(kalman_flag){
//		kalman_flag = false;
//		old_q_bias = 0;
//		old_p_bias = 0;
//		new_q_bias = 0;
//		new_p_bias = 0;
//		p2phi = float2fix(0.0023); 
//		c1 = float2fix(256);
//		c2 = float2fix(1000000);
//		old_phi = float2fix(0);
//		old_theta = float2fix(0);	
//	}
//	if(check_sensor_int_flag()){
//		get_raw_sensor_data();
//	}
//	p = fix2float(sp) - old_p_bias;
//	new_phi = old_phi + (fixmul(p,p2phi));
//	new_phi = new_phi - (new_phi - float2fix(say))/c1;
//	new_p_bias = old_p_bias + (new_phi - (float2fix(say)))/c2;
//	//p = fix2float(p);
//	//phi_error = fix2float(phi_error);
//	old_p_bias = new_p_bias;
//	old_phi = new_phi;
//
//
//	//q = sq - q_bias;
//	//est_theta = est_theta + (fixmul(q,p2phi));
//	//theta_error = est_theta - (est_theta - float2fix(sax))/c1;
//	//q_bias = q_bias + (est_theta - (float2fix(sax)/p2phi))/c2;
//	////q = fix2float(q);
//	////theta_error = fix2float(theta_error);
//
//
//	printf("new_phi: %d phi: %d\n", new_phi, phi);
//	
//}



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
		#if MOTOR_VALUES_DEBUG == 1
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d, sr:%d\n", ae[0],ae[1],ae[2],ae[3], sr);
		//printf("kp:%d\n", kp);	
		#endif
	}
	else{
		for(uint8_t i=0; i<4; i++){
			ae[i]=0;
		}
		#if MOTOR_VALUES_DEBUG == 1
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0], ae[1],ae[2],ae[3]);
		#endif
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
		roll_error = LRPY16[1]/4 - (k_LRPY[1]*4) - phi;
		pitch_error = LRPY16[2]/4 - (k_LRPY[2]*4) - theta;
		yaw_error = LRPY16[3]/4 + (k_LRPY[3]*4) + sr;								//take keyboard offset into account
		adjusted_pitch = (kp1 * pitch_error)/4 + (kp2 * sq)/2;
		adjusted_roll = (kp1 * roll_error)/4 - (kp2 * sp)/2;
		adjusted_yaw = kp * yaw_error;

		convert_to_rpm((uint16_t)LRPY16[0], adjusted_roll, adjusted_pitch, adjusted_yaw);
		#if MOTOR_VALUES_DEBUG == 1
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0], ae[1],ae[2],ae[3]);
		//printf("kp:%d,kp1:%d,kp2:%d\n", kp, kp1,kp2);
		#endif
	}else{
		for(uint8_t i=0; i<4; i++){
			ae[i]=0;
		}
		#if MOTOR_VALUES_DEBUG == 1
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0], ae[1],ae[2],ae[3]);
		#endif
	}
}

/*
*-----------------------------------------------------------------------------------------
* height_control() - Function to control the height of the drone autonomously when the drone 
* 					 enters height control mode
*
* Author: Himanshu Shah
* Date : 06/06/18
*------------------------------------------------------------------------------------------
*/
void height_control(){
	
	static int32_t height_error = 0, adjusted_lift = 0, desired_pressure = 0;
	int16_t kl = 100;
	
	for(uint8_t i =0; i<4; i++){
		LRPY16[i]=LRPY[i]<<8;
	}

	for(uint8_t i=0; i<4; i++){
			ae[i]=0;
	}

	if(LRPY[0] > 10 || LRPY[0] < -10){

		kl += k_LRPY[7];
		if(kl < 1){
			kl = 1;
			k_LRPY[7]=0;
		}

		if(check_sensor_int_flag()){
			get_dmp_data();
		}
		read_baro();
		if(height_flag<20){
			height_flag++;
			desired_pressure = pressure;
		}
		height_error = pressure - desired_pressure;
		adjusted_lift = (uint16_t)LRPY16[0] + (kl * height_error);
		convert_to_rpm((uint16_t)adjusted_lift, LRPY16[1], LRPY16[2], LRPY16[3]);
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d, kl:%d\n", ae[0], ae[1],ae[2],ae[3],kl);
		//printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0], ae[1],ae[2],LRPY[3]);
		//printf("desired: %d, pressure: %d, height_error:%d\n", desired_pressure, pressure, height_error);

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
#if CONTROLLER == 1
#define LIFT_THRESSHOLD  20
#define ROLL_THRESSHOLD  20
#define PITCH_THRESSHOLD 20
#define YAW_THRESSHOLD   20
#else
#define LIFT_THRESSHOLD  2
#define ROLL_THRESSHOLD  5
#define PITCH_THRESSHOLD 5
#define YAW_THRESSHOLD   5
#endif
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
		case SAFE_DISCONNECTED:
			//printf("S\n");
			ae[0] = 0;
			ae[1] = 0;
			ae[2] = 0;
			ae[3] = 0;
			if(USBDisconnected == false){
				QuadState = SAFE_NONZERO;
				printf("Initiate SAFE_NONZERO mode.\n");
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
				if(USBDisconnected == false){
					QuadState = SAFE_NONZERO;
					printf("Initiate SAFE_NONZERO mode.\n");
				}else{
					QuadState = SAFE_DISCONNECTED;
					printf("Initiate SAFE_DISCONNECTED mode.\n");
				}
			}
			break;
		case MANUAL:
			//Map the received values directly to the motors.
			for(uint8_t i=0; i<4; i++){
				LRPY16[i]=LRPY[i]<<8;
			}
			convert_to_rpm((uint16_t)LRPY16[0],LRPY16[1], LRPY16[2], LRPY16[3]);
			#if MOTOR_VALUES_DEBUG == 1
			printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0],ae[1],ae[2],ae[3]);
			#endif
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
			log_dump();
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
