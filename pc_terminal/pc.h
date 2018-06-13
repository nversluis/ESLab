#ifndef _PC_LIBSTUFF_H
#define _PC_LIBSTUFF_H

#include<inttypes.h>

#define JOYSTICK_HZ 100
#define PC_TERMINAL_DEBUG 1
#define PC_TERMINAL_SHOW_MOTORS 0
#define PC_TERMINAL_DISPLAY_INTERFACE_CHARS 0

int16_t ae[4];
uint16_t bat_volt;
uint8_t mode;
int16_t phi_o, theta_o, psi_o; // offsets
int16_t sp_o, sq_o, sr_o; // offsets
int16_t phi, theta, psi;
int16_t sp, sq, sr;


int rs232_getchar_nb();
void term_putchar(char c);

#endif