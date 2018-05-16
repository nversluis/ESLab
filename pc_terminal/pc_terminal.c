/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <sys/timeb.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "protocol.h"
#include "joystick.h"

int rs232_putchar(char c);


/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios 	savetty;

void	term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void	term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void	term_puts(char *s)
{
	fprintf(stderr,"%s",s);
}

void	term_putchar(char c)
{
	putc(c,stderr);
}

int	term_getchar_nb()
{
        static unsigned char 	line [2];

        if (read(0,line,1)) // note: destructive read
        		return (int) line[0];

        return -1;
}

int	term_getchar()
{
        int    c;

        while ((c = term_getchar_nb()) == -1)
                ;
        return c;
}

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */
#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>

int serial_device = 0;
int fd_RS232;

void rs232_open(void)
{
  	char 		*name;
  	int 		result;
  	struct termios	tty;

       	fd_RS232 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime

	assert(fd_RS232>=0);

  	result = isatty(fd_RS232);
  	assert(result == 1);

  	name = ttyname(fd_RS232);
  	assert(name != 0);

  	result = tcgetattr(fd_RS232, &tty);
	assert(result == 0);

	tty.c_iflag = IGNBRK; /* ignore break condition */
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
	tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */

	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 1; // added timeout

	tty.c_iflag &= ~(IXON|IXOFF|IXANY);

	result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

	tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */
}


void 	rs232_close(void)
{
  	int 	result;

  	result = close(fd_RS232);
  	assert (result==0);
}


int	rs232_getchar_nb()
{
	int 		result;
	unsigned char 	c;

	result = read(fd_RS232, &c, 1);

	if (result == 0)
		return -1;

	else
	{
		assert(result == 1);
		return (int) c;
	}
}


int 	rs232_getchar()
{
	int 	c;

	while ((c = rs232_getchar_nb()) == -1)
		;
	return c;
}


int 	rs232_putchar(char c)
{
	int result;

	do {
		result = (int) write(fd_RS232, &c, 1);
	} while (result == 0);

	assert(result == 1);
	return result;
}


/*----------------------------------------------------------------
 * main -- execute terminal
 * 
 * Mods: Himanshu Shah
 * Date: 03/05/18
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	struct timeb time_buffer;
	char	c;
	char c2;
	time_t start_time, end_time;

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");
	
	init_js();
	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");

	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);
	
	ftime(&time_buffer);
	start_time=time_buffer.time*1000 + time_buffer.millitm;
	end_time = start_time;

	/* send & receive
	 */
	for (;;)
	{
		if((start_time + 300) >= end_time){

			if ((c = term_getchar_nb()) != -1){
				//printf("Character found: %c\n", c);
				//rs232_putchar(c);

				if((int)c == 27){							 //detect for escape button and arrowkeys, as arrow keys contains escape character in them
					if((c2 = term_getchar_nb()) == -1){
						struct packet p_obj;
						p_obj.header=MODESET;
						p_obj.data=ABORT;
						//TODO: compute crc and add to packet
						p_obj.crc8=0x00;
						//TODO: implement queue to send packets
						rs232_putchar(p_obj.header);
						rs232_putchar(p_obj.data);
						rs232_putchar(p_obj.crc8);
						break;
					}
				}else{
					detect_term_input(c);
					//break;
				}
			}else{
				//printf("Nothing should be found really...\n");
			}

			if ((c = rs232_getchar_nb()) != -1){
				term_putchar(c);
			}
        	
			//send_j_packet();
		
			ftime(&time_buffer);
			end_time=time_buffer.time*1000 + time_buffer.millitm;
		
		}else{
			ftime(&time_buffer);
			start_time=time_buffer.time*1000 + time_buffer.millitm;
			end_time = start_time;
			send_j_packet();
		}
		//usleep(10000)
	}
        

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}

