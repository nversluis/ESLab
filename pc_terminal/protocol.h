//*******************Documentation Section*************************//
// This file defines the packet structure and variables            //
//                                                                 // 
// Author: Himanshu Shah & Mark Röling                             //
// Date: 03/05/18                                                  //
//*****************************************************************//

#ifndef PROTOCOL_H__
#define PROTOCOL_H__
#include<inttypes.h>

#define KEEP_ALIVE_TIMEOUT_MS 1000

void detect_term_input(char);

//     Packet Headers    //
//Mode packets
#define MODESET			0x70
#define MODEGET			0X71
#define ACK				0xAB
#define INFO			0xAC

// Control packets
#define J_CONTROL		0x80
#define J_CONTROL_D		0x81 // double precision control message
#define K_ROLL			0x82
#define K_LIFT			0x83
#define K_YAW			0x84
#define K_PITCH			0x85
#define K_P             0x86
#define K_P1            0x8A
#define K_P2            0x8B
#define K_HEIGHT        0x8C
//#define PING			0x87
#define PING			'h'
#define PING_DATCRC		0x88
#define PING_DATACK		0x89

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
#define LOG_ENTRY       0xA0
#define LOG_START       0xA1
#define LOG_END         0xA2

// Print packets
#define PRINT			0x99 // Print statement without data
#define PRINT1			0x9A // Print statement with 1 byte data
#define PRINT2			0x9B // Print statement with 2 bytes data
#define PRINT4			0x9C // Print statement with 4 bytes data
#define PRINT8			0x9D // Print statement with 4 bytes data

// Print data parameters
#define P_MAINLOOP		0x01
#define P_CHANGENOTSAFE	0x02
#define P_CALNOTSAFE	0x03
#define P_NEWMODEINFO	0x04
#define P_LOGOVERFLOW	0x05
#define P_LOGSWITCH		0x06
#define P_LOGERASEFAIL	0x07
#define P_LOGABORT		0x08
#define P_LOGDUMPABORT	0x09
#define P_LOGSPIFAIL	0x10
#define P_LOGFLASHINIT	0x11
#define P_LOGBOUNDSERR	0x12
#define P_TEST4			0x13
#define P_LOGFLASHWRITE	0x14
#define P_LOGNOTINIT	0x15
#define P_LOGENTRY		0x16
#define P_LOGEMPTYSPACE	0x17
#define P_GOODBYE		0x18
#define P_BATCRIT		0x19
#define P_BATLOW		0x20
#define P_MOTORDATA		0x21
#define P_BUTTERWORTH	0x22
#define P_NONZEROY		0x23
#define P_NONZEROP		0x24
#define P_NONZEROR		0x25
#define P_NONZEROL		0x26

//Mode values 
#define SAFE            	0x11
#define SAFE_NONZERO		0x12
#define SAFE_DISCONNECTED	0x13
#define PANIC           	0x14
#define PANIC_COUNTDOWN 	0x15
#define MANUAL          	0x16
#define CALIBRATION     	0x17
#define CALIBRATION_ENTER	0x18
#define YAWCONTROL      	0x19
#define FULLCONTROL    		0x1A
#define RAW             	0x1B
#define HEIGHT          	0x1C
#define WIRELESS        	0x1D
#define DUMPLOGS			0x1E
#define SETNEWMODE			0x2F

//Some escape value
#define ABORT			0x60

//#define INCREASEPROP    1
//#define DECREASEPROP    -1
#define DECREASE		-1
#define INCREASE		1

struct packet{
    uint8_t header;
    uint8_t data; //make it 1 byte for now, gonna fix this now
    uint8_t crc8;
};

int rs232_putchar(char c);
void term_puts(char *s);


#endif   // protocol.h