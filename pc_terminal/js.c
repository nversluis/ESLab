
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#include "joystick.h"
#include "../crc.h"
#include "protocol.h"

#define JS_DEV	"/dev/input/js0"


/* current axis and button readings
 */

int fd;
struct js_event js;
struct packet j_obj;
int	axis[6];
char new_axis[6];
int	button[12];


unsigned int    mon_time_ms(void)
{
        unsigned int    ms;
        struct timeval  tv;
        struct timezone tz;

        gettimeofday(&tv, &tz);
        ms = 1000 * (tv.tv_sec % 65); // 65 sec wrap around
        ms = ms + tv.tv_usec / 1000;
        return ms;
}

void j_scale(int axis[])
{

	unsigned int range = 65535;
	unsigned int new_range = 255;
	unsigned int max = 32767;
	int min = -32768;
	int new_min = -128;
	
	for(int i = 0; i < 6; i++)
	{		
		if(axis[i] >= min && axis[i] <= max)
		{
			if (axis[i] == 0) 
			{
				new_axis[i] = 0;
			} 
			else 
			{
				new_axis[i] = (((axis[i] - min) * new_range) / range) + new_min;
			}
		}
	}
}

void    mon_delay_ms(unsigned int ms)
{
        struct timespec req, rem;

        req.tv_sec = ms / 1000;
        req.tv_nsec = 1000000 * (ms % 1000);
        assert(nanosleep(&req,&rem) == 0);
}

void init_js(){

	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		exit(1);
	}

	/* non-blocking mode
	 */
	fcntl(fd, F_SETFL, O_NONBLOCK);
}

void make_j_packet(){
	j_obj.header = J_CONTROL;
	j_obj.crc8 = 0x00;
	rs232_putchar(j_obj.header);
	for (uint8_t i = 0; i < 4; i++) {
		rs232_putchar(new_axis[i]);
	}
	rs232_putchar(j_obj.crc8);
	

		//printf("\n<exit>\n");
	}


void send_j_packet()
{
	

	//while (1) {


		/* simulate work
		 */
	//	mon_delay_ms(300);
	//	t = mon_time_ms();

		/* check up on JS
		 */
	while (read(fd, &js, sizeof(struct js_event)) == 
	       			sizeof(struct js_event))  {
		/* register data
		 */
		// fprintf(stderr,".");
		switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				button[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				axis[js.number] = js.value;
				break;
		}
	}
	if (errno != EAGAIN) {
		perror("\njs: error reading (EAGAIN)");
		exit (1);
	}
	
	j_scale(&axis[0]);									//scale down joystick axes values to one byte

	if (button[0] || button[1]){
		term_puts("Entering PANIC mode.....");			//If safe mode or panic mode button pressed, go to panic mode(which will automatically end up in safe mode)			
		j_obj.header=MODESET;
		j_obj.data=PANIC;
		j_obj.crc8=0x00;								//TODO: compute CRC using function
		rs232_putchar(j_obj.header);
		rs232_putchar(j_obj.data);
		rs232_putchar(j_obj.crc8);
		return;;											//TODO: convert into a function and replace with a return 
	}

	if(abs(axis[0] >= 20)){
		make_j_packet();
	}
	
}
