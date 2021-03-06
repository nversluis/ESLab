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
		if(rotor[i] >= 1000){
			rotor[i] = 1000;
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

    static bool butter_flag=true;
    static int32_t x[3] = {0,0,0}, y[3] = {0,0,0}, filtered_sr, a0, a1, a2, b0, b1;
    if(butter_flag){
		//200Hz
        a0=float2fix(0.0200833656);
        a1=float2fix(0.0401667312);
        a2=float2fix(0.0200833656);
        b0=float2fix(-0.6413515381);
        b1=float2fix(1.5610180758);
		//100Hz
		// a0=float2fix(0.0674552739);
        // a1=float2fix(0.134910548);
        // a2=float2fix(0.0674552739);
        // b0=float2fix(-0.4128015981);
        // b1=float2fix(1.1429805025);
        butter_flag=false;
    }
	if (check_sensor_int_flag()) 
	{
		get_raw_sensor_data();
	}
	
    x[0] = x[1];
    x[1] = x[2];
    x[2] = sr;
    y[0] = y[1];
    y[1] = y[2];
    filtered_sr = (a0*x[0]) + (a2*x[2]) + (a1*x[1]) + (b0 * y[0]) + (b1 * y[1]);
    filtered_sr = fix2float(filtered_sr);
    y[2] = filtered_sr;

	uint32_t tdata[2];
	tdata[0] = (uint32_t)sr;
	tdata[1] = (uint32_t)filtered_sr;
	remote_print_data(P_BUTTERWORTH, sizeof(tdata), (uint8_t*)tdata);
	
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

void kalman_filter(){
	static bool kalman_flag = true;
	static int32_t sp_temp, sq_temp, say_temp, sax_temp, sp_kalman, sq_kalman, phi_kalman, theta_kalman, phi_error, theta_error, p_bias, q_bias, phi_raw, theta_raw, sp_raw, sq_raw, p2phi, c1, c2; 
	if(kalman_flag){
		imu_init(false,100);
		kalman_flag = false;
		q_bias = 0;
		p_bias = 0;
		p2phi = float2fix(0.0081); 
		phi_kalman = 0;
		theta_kalman = 0;
		c1 = 128;
		c2 = 800000;
	}

	if(check_sensor_int_flag()){
		get_raw_sensor_data();
	}
	// if(flag){
	// 	if(counter%10 == 0){
	// 		flag = false;
	// 		control_time = get_time_us();
	// 		control_time = control_time - start_time;
	// 		control_time = control_time/10;
	// 	}
	// 	else{
	// 		counter++;
	// 	}
	// // start_time = get_time_us();

	sp_temp = float2fix(sp);
	say_temp = float2fix(say);
	sp_kalman = sp_temp - p_bias;
	phi_kalman = phi_kalman + fixmul(sp_kalman,p2phi);
	phi_error = (phi_kalman - say_temp);
	phi_kalman = phi_kalman - (phi_error/c1);
	p_bias = p_bias + (float2fix((phi_error/p2phi))/c2);
	phi_raw = fix2float(phi_kalman);
	sp_raw = fix2float(sp_kalman);
	
	sq_temp = float2fix(sq);
	sax_temp = float2fix(sax);
	sq_kalman = sq_temp - q_bias;
	theta_kalman = theta_kalman + fixmul(sq_kalman,p2phi);
	theta_error = (theta_kalman - sax_temp);
	theta_kalman = theta_kalman - (theta_error/c1);
	q_bias = q_bias + (float2fix((theta_error/p2phi))/c2);
	theta_raw = fix2float(theta_kalman);
	sq_raw = fix2float(sq_kalman);
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

	for(uint8_t i =0; i<4; i++){
		LRPY16[i]=((int16_t)LRPY[i])<<8;
	}

	if((uint16_t)LRPY[0] > 30){
		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
		}
		
		kp += k_LRPY[4];
		if(kp < 1){
			kp = 1;
			k_LRPY[4]=kp-10;
		}
		if(kp > 50){
			kp=50;
			k_LRPY[4]=kp-10;
		}
		yaw_error = LRPY16[3]/4 + k_LRPY[3]*4 + (sr - sr_o);								//take keyboard offset into account
		adjusted_yaw = (kp * yaw_error)/4;

		convert_to_rpm((uint16_t)LRPY16[0], LRPY16[1], LRPY16[2], adjusted_yaw);
		#if MOTOR_VALUES_DEBUG == 1
		send_motor_data();
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d, kp:%d\n", ae[0],ae[1],ae[2],ae[3],kp);
		#endif
	}
	else{
		for(uint8_t i=0; i<4; i++){
			ae[i]=0;
		}
		#if MOTOR_VALUES_DEBUG == 1
		send_motor_data();
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d, kp:%d\n", ae[0], ae[1],ae[2],ae[3], kp);
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
	int32_t roll_error, pitch_error, yaw_error, adjusted_roll, adjusted_pitch, adjusted_yaw;
	int8_t kp = 10,kp1 = 8,kp2 = 20;

	for(int8_t i=0; i<4; i++){
		LRPY16[i] = ((int16_t)(LRPY[i]))<<8;
	}

	if((uint8_t)LRPY[0] > 30){
		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
		}
		
		kp += k_LRPY[4];
		if(kp < 1){
			kp = 1;
			k_LRPY[4]=kp-10;
		}
		if(kp > 50){
			kp=50;
			k_LRPY[4]=kp-10;
		}
		kp1 += k_LRPY[5];
		if(kp1 < 1){
			kp1 = 1;
			k_LRPY[5]=kp1-8;
		}
		if(kp1 > 50){
			kp1 = 50;
			k_LRPY[5]=kp1-8;
		}
		kp2 += k_LRPY[6];
		if(kp2 < 1 ){
			kp2 = 1;
			k_LRPY[6]=kp2-20;
		}
		if(kp2 > 50){
			kp2 = 50;
			k_LRPY[6]=kp2-20;
		}
		roll_error = LRPY16[1]/4 - (k_LRPY[1]*4) - (phi - phi_o);
		pitch_error = LRPY16[2]/4 - (k_LRPY[2]*4) - (theta - theta_o);
		yaw_error = LRPY16[3]/4 + (k_LRPY[3]*4) + (sr - sr_o);								//take keyboard offset into account
		adjusted_pitch = (kp1 * pitch_error)/4 + (kp2 * (sq - sq_o))/2;
		adjusted_roll = (kp1 * roll_error)/4 - (kp2 * (sp - sp_o))/2;
		adjusted_yaw = (kp * yaw_error)/4;

		convert_to_rpm((uint16_t)LRPY16[0], adjusted_roll, adjusted_pitch, adjusted_yaw);
		#if MOTOR_VALUES_DEBUG == 1
		send_motor_data();
		printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d, kp:%d, kp1:%d, kp2:%d\n", ae[0], ae[1],ae[2],ae[3], kp,kp1,kp2);
		//printf("kp:%d,kp1:%d,kp2:%d\n", kp, kp1,kp2);
		#endif
	}else{
		for(uint8_t i=0; i<4; i++){
			ae[i]=0;
		}
		#if MOTOR_VALUES_DEBUG == 1
		send_motor_data();
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
	static uint8_t height_flag=0;
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
		// What is kl?
		send_motor_data();
		//printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d, kl:%d\n", ae[0], ae[1],ae[2],ae[3],kl);
		//printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0], ae[1],ae[2],LRPY[3]);
		//printf("desired: %d, pressure: %d, height_error:%d\n", desired_pressure, pressure, height_error);

	}
	else{
		for(uint8_t i=0; i<4; i++){
			ae[i]=0;
		}
		send_motor_data();
		//printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0], ae[1],ae[2],ae[3]);	
		
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
					remote_print_data(P_NONZEROY, sizeof(uint8_t), (uint8_t*)&LRPY[3]);
					//printf("Yaw non-zero: %d\n", (int8_t)LRPY[3]);
					#endif
				}
			}else{
				#if NON_ZERO_DEBUG == 1
				remote_print_data(P_NONZEROP, sizeof(uint8_t), (uint8_t*)&LRPY[2]);
				//printf("Pitch non-zero: %d\n", (int8_t)LRPY[2]);
				#endif
			}
		}else{
			#if NON_ZERO_DEBUG == 1
			remote_print_data(P_NONZEROR, sizeof(uint8_t), (uint8_t*)&LRPY[1]);
			//printf("Roll non-zero: %d\n", (int8_t)LRPY[1]);
			#endif
		}
	}else{
		#if NON_ZERO_DEBUG == 1
		remote_print_data(P_NONZEROL, sizeof(uint8_t), (uint8_t*)&LRPY[0]);
		//printf("Lift non-zero: %d\n", (uint8_t)LRPY[0]);
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
	#if DEBUG_STATE_TRANSITIONS == 1
	uint8_t datat[2];
	#endif

	switch(QuadState){
		case SAFE:
			//printf("S\n");
			ae[0] = 0;
			ae[1] = 0;
			ae[2] = 0;
			ae[3] = 0;
			if(!near_zero()){ //inputs are not near 0
				//printf("Safe mode, non-zero.\n");
				remote_notify_state(SAFE_NONZERO, INFO);
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
				//printf("Safe mode, zero.\n");
				remote_notify_state(SAFE, INFO);
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
				//printf("Initiate SAFE_NONZERO mode.\n");
				remote_notify_state(SAFE_NONZERO, INFO);
			}
			break;
		case PANIC:
			//printf("Initiate PANIC mode.\n");
			remote_notify_state(PANIC, INFO);

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
					remote_notify_state(SAFE_NONZERO, INFO);
					//printf("Initiate SAFE_NONZERO mode.\n");
				}else{
					QuadState = SAFE_DISCONNECTED;
					remote_notify_state(SAFE_DISCONNECTED, INFO);
					//printf("Initiate SAFE_DISCONNECTED mode.\n");
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
			void send_motor_data();
			//remote_print_data(P_MOTORDATA, sizeof(ae), ae);
			//printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0],ae[1],ae[2],ae[3]);
			#endif
			break;
		case CALIBRATION_ENTER:
			//printf("Initiate CALIBRATION mode.\n");
			
			CalibrationStartTime = get_time_us();
			phi_o = phi;
			theta_o = theta;
			psi_o = psi;
			sp_o = sp;
			sq_o = sq;
			sr_o = sr;
			QuadState = CALIBRATION;
			remote_notify_state(CALIBRATION, INFO);
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
				send_calibration_data();
				//printf("Calibration done. Offsets: phi=%d, theta=%d, psi=%d, sp=%d, sq=%d, sr=%d.\n", phi_o, theta_o, psi_o, sp_o, sq_o, sr_o);
				QuadState = PANIC;
				remote_notify_state(PANIC, INFO);
			}
			break;
		case YAWCONTROL:
			yaw_control();
			break;
		case FULLCONTROL:
			full_control();
			break;
		case RAW:
			butterworth_filter();
			//remote_notify_state(PANIC, INFO);
			break;
		case HEIGHT:
			QuadState = PANIC;
			remote_notify_state(PANIC, INFO);
			break;
		case WIRELESS:
			QuadState = PANIC;
			remote_notify_state(PANIC, INFO);
			break;
		case DUMPLOGS:
			log_dump();
			QuadState = SAFE;
			remote_notify_state(SAFE, INFO);
			break;
		case SETNEWMODE:
			#if DEBUG_STATE_TRANSITIONS == 1
			// Debug mode transitions
			datat[0] = PreviousMode;
			datat[1] = ModeToSet;
			remote_print_data(P_NEWMODEINFO, sizeof(PreviousMode)+sizeof(ModeToSet), datat);
			#endif

			// Do nothing if we want the same mode again.
			if(PreviousMode == ModeToSet){
				QuadState = PreviousMode;
				break;
			}

			// Don't allow any mode changes from PANIC or SAFE_NONZERO.
			if(PreviousMode == PANIC || PreviousMode == PANIC_COUNTDOWN || PreviousMode == SAFE_NONZERO){
				// printf("Can't statechange out of protected modes.\n");
				QuadState = PreviousMode;
				break;
			}

			// Check full to heigth and vice-versa
			if(ModeToSet == FULLCONTROL){
				if(PreviousMode == HEIGHT || PreviousMode == SAFE){
					//printf("FULLCONTROL set.\n");
					QuadState = ModeToSet;
					remote_notify_state(ModeToSet, ACK);
					break;
				}else{
					remote_print(P_CHANGENOTSAFE);
					//printf("Cannot change flight modes, you're not in SAFE mode.\n");
					QuadState = PreviousMode;
					break;
				}
			}
			if(ModeToSet == HEIGHT){
				if(PreviousMode == FULLCONTROL || PreviousMode == SAFE){
					//printf("HEIGHT set.\n");
					remote_notify_state(ModeToSet, ACK);
					QuadState = ModeToSet;
					break;
				}else{
					remote_print(P_CHANGENOTSAFE);
					//printf("Cannot change flight modes, you're not in SAFE mode.\n");
					QuadState = PreviousMode;
					break;
				}
			}

			// Check Flights only from safe
			if(ModeToSet == MANUAL || ModeToSet == YAWCONTROL || ModeToSet == RAW){
				if(PreviousMode == SAFE){
					//printf("MANUAL or YAWCONTROL set.\n");
					remote_notify_state(ModeToSet, ACK);
					QuadState = ModeToSet;
					break;
				}else{
					remote_print(P_CHANGENOTSAFE);
					//printf("Cannot change flight modes, you're not in SAFE mode.\n");
					QuadState = PreviousMode;
					break;
				}
			}

			// Only go to DUMPLOGS and CALIBRATION from SAFE modes.
			if((ModeToSet == DUMPLOGS || ModeToSet == CALIBRATION )){
				if(PreviousMode == SAFE || PreviousMode == SAFE_NONZERO){
					QuadState = ModeToSet;
					if(ModeToSet == CALIBRATION){
						remote_notify_state(CALIBRATION_ENTER, ACK);
						//printf("CALIBRATION mode set.\n");
						QuadState = CALIBRATION_ENTER;
					}else{
						remote_notify_state(ModeToSet, ACK);
						//printf("DUMPLOGS mode set.\n");
					}
					break;
				}else{
					remote_print(P_CALNOTSAFE);
					//printf("Cannot change to logs/calibration modes, you're not in SAFE mode.\n");
					QuadState = PreviousMode;
					break;
				}
			}

			// Something went wrong
			//remote_notify_state(PANIC, ACK);
			QuadState = PANIC;
			break;
		default:
			QuadState = PANIC;
			break;


	}
	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}
