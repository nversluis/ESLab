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
#define J_CONTROL_D		0x85 // double precision control message
#define K_ROLL			0x81
#define K_LIFT			0x82
#define K_YAW			0x83
#define K_PITCH			0x84

// Logging packets
#define SYSTIME			0x90
#define AE_OUT			0x91
#define BAT				0x92
#define PRESSURE		0x93
#define GYRO_OUT		0x94
#define ACCEL_OUT		0x95
#define CON_PARAM		0x96
#define CAL_GET			0x97
	



//Mode values 
#define SAFE            0x11
#define PANIC           0x12
#define MANUAL          0x13
#define CALIBRATION     0x14
#define YAWCONTROL      0x15
#define FULLCONTROL     0x16
#define RAW             0x17
#define HEIGHT          0x18
#define WIRELESS        0x19
#define ABORT           0x60




#define DECREASE		-1
#define INCREASE		1

struct packet{
    uint8_t header;
    uint8_t data; //make it 1 byte for now, gonna fix this later
    uint8_t crc8;
};

int rs232_putchar(char c);
void term_puts(char *s);


#endif   // protocol.h