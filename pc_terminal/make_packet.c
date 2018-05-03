//*******************Documentation Section*************************
// This file maps the detected key from keyboard and builds a      
// corresponding packet and sends it to the quadcopter              
// 
// Author: Himanshu Shah                                           
// Date: 03/05/18                                                  
//*****************************************************************

#include<stdio.h>
#include "protocol.h"

void detect_term_input(char c){

    struct packet p_obj;
    int data_detected=0;

    switch(c)
    {
        case '0':
                 p_obj.header=MODESET;
                 p_obj.data=SAFE;
                 data_detected=1;
                 break;
        case '1':
                 p_obj.header=MODESET;
                 p_obj.data=PANIC;
                 data_detected=1;
                 break;
        case '2': 
                 p_obj.header=MODESET;
                 p_obj.data=MANUAL;
                 data_detected=1;
                 break;
        case '3':
                 p_obj.header=MODESET;
                 p_obj.data=CALIBRATION;
                 data_detected=1;
                 break;
        case '4':
                 p_obj.header=MODESET;
                 p_obj.data=YAWCONTROL;
                 data_detected=1;
                 break;
        case '5':
                 p_obj.header=MODESET;
                 p_obj.data=FULLCONTROL;
                 data_detected=1;
                 break;
        case '6':
                 p_obj.header=MODESET;
                 p_obj.data=RAW;
                 data_detected=1;
                 break;
        case '7':
                 p_obj.header=MODESET;
                 p_obj.data=HEIGHT;
                 data_detected=1;
                 break;
        case '8':
                 p_obj.header=MODESET;
                 p_obj.data=WIRELESS;
                 data_detected=1;
                 break;
        case 'a':
                 p_obj.header=K_LIFT;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'z':
                 p_obj.header=K_LIFT;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'q':
                 p_obj.header=K_YAW;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'w':
                 p_obj.header=K_YAW;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'A':
                 p_obj.header=K_PITCH;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        case 'B':
                 p_obj.header=K_PITCH;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'C':
                 p_obj.header=K_ROLL;
                 p_obj.data=INCREASE;
                 data_detected=1;
                 break;
        case 'D':
                 p_obj.header=K_ROLL;
                 p_obj.data=DECREASE;
                 data_detected=1;
                 break;
        default: 
                break;
    }
    if(data_detected == 1){
        //TODO: compute CRC and put in packet
        p_obj.crc8=0x00;
        //TODO: crude method, make use of rs232 queue instead of this 
        rs232_putchar(p_obj.header);
        rs232_putchar(p_obj.data);
        rs232_putchar(p_obj.crc8);
    }

}