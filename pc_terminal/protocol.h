//*******************Documentation Section*************************//
// This file defines the packet structure and variables            //
//                                                                 // 
// Author: Himanshu Shah & Mark Röling                             //
// Date: 03/05/18                                                  //
//*****************************************************************//

#ifndef PROTOCOL_H__
#define PROTOCOL_H__
#include<inttypes.h>


void detect_term_input(char);

//     Packet Headers    //
//Mode packets
#define MODESET			0x70
#define MODEGET			0X71

// Control packets
#define J_CONTROL		0x80
#define J_CONTROL_D		0x81 // double precision control message
#define K_ROLL			0x82
#define K_LIFT			0x83
#define K_YAW			0x84
#define K_PITCH			0x85
#define K_YAWP          0x86
//#define PING			0x87
#define PING			'h'
#define PING_DATCRC		0x88

// Logging packets
#define SYSTIME			0x90
#define AE_OUT			0x91
#define BAT				0x92
#define PRESSURE		0x93
#define GYRO_OUT		0x94
#define ACCEL_OUT		0x95
#define CON_PARAM		0x96
#define CAL_GET			0x97
#define BIG_PACKET		0x98
	



//Mode values 
#define SAFE            	0x11
#define SAFE_NONZERO		0x12
#define PANIC           	0x13
#define PANIC_COUNTDOWN 	0x14
#define MANUAL          	0x15
#define CALIBRATION     	0x16
#define CALIBRATION_ENTER	0x1D
#define YAWCONTROL      	0x17
#define FULLCONTROL    		0x18
#define RAW             	0x19
#define HEIGHT          	0x1A
#define WIRELESS        	0x1B
#define DUMPLOGS			0x1C
#define SETNEWMODE			0x2F

//Some escape value
#define ABORT			0x60

#define INCREASEPROP    1
#define DECREASEPROP    -1
#define DECREASE		-10
#define INCREASE		10

struct packet{
    uint8_t header;
    uint8_t data; //make it 1 byte for now, gonna fix this later
    uint8_t crc8;
};

int rs232_putchar(char c);
void term_puts(char *s);


#endif   // protocol.h