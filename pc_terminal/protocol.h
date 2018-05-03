//*******************Documentation Section*************************//
// This file defines the packet structure and variables            //
//                                                                 // 
// Author: Himanshu Shah                                           //
// Date: 03/05/18                                                  //
//*****************************************************************//

#include<inttypes.h>

//packet types or headers
#define MODE    	    0x70
#define LOGGING         0x71
#define J_ROLL          0x72
#define J_LIFT          0x73
#define J_YAW           0x74
#define J_PITCH	        0x75
#define K_ROLL	        0x76
#define K_LIFT	        0x77
#define K_YAW	        0x78
#define K_PITCH	        0x79
	
	

//Data values 
#define SAFE            0x11
#define PANIC           0x12
#define MANUAL          0X13
#define CALIBRATION     0X14
#define YAWCONTROL      0X15
#define FULLCONTROL     0X16
#define RAW             0X17
#define HEIGHT          0X18
#define WIRELESS        0X19
#define ABORT           0x60
#define INCREASE        0x01
#define DECREASE        0x02

struct packet{
    char header;
    signed data;
    char crc8;
};