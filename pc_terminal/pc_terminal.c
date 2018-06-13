/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <stdbool.h>
#include <sys/timeb.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "protocol.h"
#include "joystick.h"
#include "../crc.h"
#include "pc.h"

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

	cfsetospeed(&tty, B921600); // B115200 B921600
	cfsetispeed(&tty, B921600); // B115200

	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 0; // added timeout

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
 * printMode -- prints the relevant mode to terminal
 * 
 * Mods: Mark Röling
 * Date: 07/06/18
 *----------------------------------------------------------------
 */
void printMode(uint8_t mode){
	switch(mode){
		case SAFE:
			printf("SAFE");
			break;
		case SAFE_NONZERO:
			printf("SAFE_NONZERO");
			break;
		case SAFE_DISCONNECTED:
			printf("SAFE_DISCONNECTED");
			break;
		case PANIC:
			printf("PANIC");
			break;
		case PANIC_COUNTDOWN:
			printf("PANIC_COUNTDOWN");
			break;
		case MANUAL:
			printf("MANUAL");
			break;
		case CALIBRATION:
			printf("CALIBRATION");
			break;
		case CALIBRATION_ENTER:
			printf("CALIBRATION_ENTER");
			break;
		case YAWCONTROL:
			printf("YAWCONTROL");
			break;
		case FULLCONTROL:
			printf("FULLCONTROL");
			break;
		case RAW:
			printf("RAW");
			break;
		case HEIGHT:
			printf("HEIGHT");
			break;
		case WIRELESS:
			printf("WIRELESS");
			break;
		case DUMPLOGS:
			printf("DUMPLOGS");
			break;
		case SETNEWMODE:
			printf("SETNEWMODE");
			break;
		default:
			printf("Unknown: 0x%02X", mode);
			break;
	}
}


/*----------------------------------------------------------------
 * process_packet -- the computer side state machine
 * 
 * Mods: Mark Röling
 * Date: 07/06/18
 *----------------------------------------------------------------
 */
#define MAX_PACKET_SIZE 10
uint8_t inPacketState = 0;
uint8_t headerByte = 0x00;
uint8_t totalBytesToRead = 0;
uint8_t inPacketBuffer[MAX_PACKET_SIZE];
uint8_t inPacketBufSize = 0;
#define PACKET_DEBUG 0

void process_packet(uint8_t readByte){
	bool CRCIsValid = false;
	uint8_t headerFound = false;
	uint8_t crc_calc = 0;
	int16_t offsets[6];

	#if PACKET_DEBUG == 1
	printf("Comp: Readbyte: 0x%02X\n", readByte);
	#endif
	/*
	if(readByte == PING){
		//uart_put(PING);
		printf("Quad Ack: Ping.\n");
		inPacketState = 0;
	}
	*/
	
	switch(inPacketState){
		case 0:
			if(readByte == MODESET || readByte == MODEGET || readByte == K_ROLL || readByte == K_LIFT || readByte == K_YAW || readByte == K_P || readByte == K_P1 || readByte == K_P2 || readByte == K_HEIGHT || readByte == K_PITCH || readByte == PING_DATCRC || readByte == PRINT){
				// 1 Byte packets
				headerByte = readByte;
				totalBytesToRead = 2;
				headerFound = true;
			}
			else if(readByte == BAT || readByte == PRINT1){
				// 2 Byte packets
				headerByte = readByte;
				totalBytesToRead = 3;
				headerFound = true;
			}
			else if(readByte == PRINT2){
				// 3 Byte packets
				headerByte = readByte;
				totalBytesToRead = 4;
				headerFound = true;
			}
			else if(readByte == J_CONTROL || readByte == SYSTIME || readByte == PRESSURE){
				// 4 Byte packets
				headerByte = readByte;
				totalBytesToRead = 5;
				headerFound = true;
			}
			else if(readByte == PRINT4){
				// 5 Byte packets
				headerByte = readByte;
				totalBytesToRead = 6;
				headerFound = true;
			}
			else if(readByte == J_CONTROL_D || readByte == AE_OUT || readByte == GYRO_OUT || readByte == CAL_GET){
				// 8 Byte packets
				headerByte = readByte;
				totalBytesToRead = 9;
				headerFound = true;
			}

			if(headerFound == true){
				inPacketBufSize = 0;
				inPacketState = 1;
				#if PACKET_DEBUG == 1
				printf("Comp: Header byte found: 0x%02X\n", headerByte);
				#endif
			}else{
				#if PACKET_DEBUG == 1
				printf("Comp: Non-header found:  0x%02X\n", readByte);
				#endif
			}
			break;
		case 1:
			inPacketBuffer[inPacketBufSize++] = readByte;
			
			if(inPacketBufSize >= totalBytesToRead){
				inPacketState = 2;
				#if PACKET_DEBUG == 1
				printf("Comp: CRC byte found:    0x%02X ", readByte);
				#endif
			}else{
				#if PACKET_DEBUG == 1
				printf("Comp: Data byte found:   0x%02X\n", readByte);
				#endif
				break;
			}
		case 2:
			//printf("Calculating CRC... Headerbyte: %02X, inPacketBuffer[0]: %02X, inPacketBufSize: %02X\n", headerByte, inPacketBuffer[0], inPacketBufSize);
			crc_calc = make_crc8_tabled(headerByte, (uint8_t*)inPacketBuffer, inPacketBufSize-1);
			if(crc_calc == inPacketBuffer[inPacketBufSize-1]){
				CRCIsValid = true;
				#if PACKET_DEBUG == 1
				printf("- Valid.\n");
				#endif
			}else{
				CRCIsValid = false;
				#if PACKET_DEBUG == 1
				printf("- Invalid! Calculated CRC %02X, but got %02X\n", crc_calc, inPacketBuffer[inPacketBufSize-1]);
				#endif
			}
			#if PACKET_DEBUG == 1
			printf("Comp: Total data bytes read: %d\n", inPacketBufSize-1);
			#endif
			if(CRCIsValid == true){
				switch(headerByte){
					case MODESET:
						mode = inPacketBuffer[0];
						#if PC_TERMINAL_DEBUG == 1
						printf("Quad Ack: Modeset: ");
						printMode(inPacketBuffer[0]);
						printf("\n");
						#endif
					case MODEGET:
						mode = inPacketBuffer[0];
						#if PC_TERMINAL_DEBUG == 1
						printf("Quad: Modeget: ");
						printMode(inPacketBuffer[0]);
						printf("\n");
						#endif
						break;
					case BAT:
						bat_volt = (uint16_t)(inPacketBuffer[0] | inPacketBuffer[1] << 8);
						#if PC_TERMINAL_DEBUG == 1
						printf("Quad: Battery voltage: %u volts.\n", bat_volt);
						#endif
						break;
					case CAL_GET:
						for(uint8_t i=0; i<6; i++){
							offsets[i] = (int16_t)((int16_t)inPacketBuffer[2*i] | (int16_t)inPacketBuffer[(2*i)+1]<<8);
						}
						phi_o = offsets[0];
						theta_o = offsets[1];
						psi_o = offsets[2];
						sp_o = offsets[3];
						sq_o = offsets[4];
						sr_o = offsets[5];
						#if PC_TERMINAL_DEBUG == 1
						printf("Quad: Calibration Offsets: phi=%d, theta=%d, psi=%d, sp=%d, sq=%d, sr=%d.\n", phi_o, theta_o, psi_o, sp_o, sq_o, sr_o);
						#endif
						break;
					case AE_OUT:
						for(uint8_t i=0; i<4; i++){
							ae[i] = (int16_t)((int16_t)inPacketBuffer[2*i] | (int16_t)inPacketBuffer[(2*i)+1]<<8);
						}
						#if PC_TERMINAL_SHOW_MOTORS == 1
						printf("Quad: ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", ae[0],ae[1],ae[2],ae[3]);
						#endif
						break;
					/*
					case J_CONTROL:
						printf("Quad Ack: Lift: %d, Roll: %d, Pitch: %d, Yaw: %d\n", (uint8_t)inPacketBuffer[0], (int8_t)inPacketBuffer[1], (int8_t)inPacketBuffer[2], (int8_t)inPacketBuffer[3]);
						break;
					case K_LIFT:
						printf("Quad Ack: k_LRPY[0]: 0x%02X\n", (int8_t)inPacketBuffer[0]);
						break;
					case K_ROLL:
						printf("Quad Ack: k_LRPY[1]: 0x%02X\n", (int8_t)inPacketBuffer[1]);
						break;
					case K_PITCH:
						printf("Quad Ack: k_LRPY[2]: 0x%02X\n", (int8_t)inPacketBuffer[2]);
						break;
					case K_YAW:
						printf("Quad Ack: k_LRPY[3]: 0x%02X\n", (int8_t)inPacketBuffer[3]);
						break;
					case K_P:
						printf("Quad Ack: k_LRPY[4]: 0x%02X\n", (int8_t)inPacketBuffer[4]);
						break;
					case K_P1:
						printf("Quad Ack: k_LRPY[5]: 0x%02X\n", (int8_t)inPacketBuffer[5]);
						break;
					case K_P2:
						printf("Quad Ack: k_LRPY[6]: 0x%02X\n", (int8_t)inPacketBuffer[6]);
						break;
					case K_HEIGHT:
						printf("Quad Ack: k_LRPY[7]: 0x%02X\n", (int8_t)inPacketBuffer[7]);
						break;
					*/
					case PING_DATCRC:
						printf("Quad: Pingdata? It's not supposed to send this to the computer...\n");
						break;
					case PING_DATACK:
						printf("Quad Ack: Pingdata.\n");
						break;
					case PRINT:
						printf("Quad: ");
						switch(inPacketBuffer[0]){
							case P_MAINLOOP:
								printf("Entering main loop...\n");
								break;
							case P_CHANGENOTSAFE:
								printf("Cannot change flight modes, you're not in SAFE mode.\n");
								break;
							case P_CALNOTSAFE:
								printf("Cannot change to logs/calibration modes, you're not in SAFE mode.\n");
								break;
							case P_LOGOVERFLOW:
								printf("WARNING: Flash overflow detected: old data will be erased!\n");
								break;
							case P_LOGERASEFAIL:
								printf("ERROR: 4k Sector erase failed.\n");
								break;
							case P_LOGABORT:
								printf("ERROR: Logging aborted!\n");
								break;
							case P_LOGDUMPABORT:
								printf("ERROR: Log dump aborted early.\n");
								break;
							case P_LOGSPIFAIL:
								printf("ERROR: SPI Flash initialization failed\n");
								break;
							case P_LOGFLASHINIT:
								printf("ERROR: Flash not initalized yet!");
								break;
							case P_LOGFLASHWRITE:
								printf("ERROR: Flash write failed\n");
								break;
							case P_LOGNOTINIT:
								printf("ERROR: Log not initialized\n");
								break;
							case P_LOGEMPTYSPACE:
								printf("INFO: Empty flash space found.\n");
								break;
							case P_GOODBYE:
								printf("Goodbye.\n");
								break;
							case 0x00:
							default:
								printf("Generic error.\n");
								break;
						}
						break;
					case PRINT1:
						printf("Quad: ");
						switch(inPacketBuffer[0]){
							case P_LOGSWITCH:
								printf("Switching flash block to %u\n", inPacketBuffer[1]);
								break;
							case 0x00:
							default:
								printf("Generic error with data: 0x%02X.\n", inPacketBuffer[1]);
								break;
						}
						break;
					case PRINT2:
						switch(inPacketBuffer[0]){
							case P_NEWMODEINFO:
								#if PC_TERMINAL_DEBUG == 1
								printf("SETNEWMODE: previousmode: ");
								printMode(inPacketBuffer[1]);
								printf(", modetoset: ");
								printMode(inPacketBuffer[2]);
								printf(".\n");
								#endif
								break;
							case P_BATCRIT:
								bat_volt = (uint16_t)(inPacketBuffer[1] | inPacketBuffer[2] << 8);
								printf("Quad: Battery Critically low (%u volts)!\n", bat_volt);
								break;
							case P_BATLOW:
								bat_volt = (uint16_t)(inPacketBuffer[1] | inPacketBuffer[2] << 8);
								printf("Quad: Battery low (%u volts)!\n", bat_volt);
								break;
							case 0x00:
							default:
								printf("Quad: Generic error with data: 0x%02X, 0x%02X.\n", inPacketBuffer[1], inPacketBuffer[2]);
								break;
						}
						break;
					case PRINT4:
						printf("Quad: ");
						switch(inPacketBuffer[0]){
							case P_LOGBOUNDSERR:
								printf("ERROR: Address(0x%08X) + log size out of bounds\n", (uint32_t)(inPacketBuffer[1] | inPacketBuffer[2] << 8 | inPacketBuffer[3] << 16 | inPacketBuffer[4] << 24));
								break;
							case P_TEST4:
								printf("Test variable: 0x%08X.\n", (uint32_t)(inPacketBuffer[1] | inPacketBuffer[2] << 8 | inPacketBuffer[3] << 16 | inPacketBuffer[4] << 24));
								break;
							case P_LOGENTRY:
								printf("Entry(0x%08X).\n", (uint32_t)(inPacketBuffer[1] | inPacketBuffer[2] << 8 | inPacketBuffer[3] << 16 | inPacketBuffer[4] << 24));
							case 0x00:
							default:
								printf("Generic error with data: 0x%02X, 0x%02X, 0x%02X, 0x%02X.\n", inPacketBuffer[1], inPacketBuffer[2], inPacketBuffer[3], inPacketBuffer[4]);
								break;
						}
						break;
					case PRINT8:
						printf("Quad: ");
						switch(inPacketBuffer[0]){
							/*
							case P_MOTORDATA:
								for(uint8_t i=0; i<4; i++){
									aet[i] = (int16_t)((int16_t)inPacketBuffer[2*i] | (int16_t)inPacketBuffer[(2*i)+1]<<8);
								}
								printf("ae0:%d, ae1:%d, ae2:%d, ae3:%d\n", aet[0],aet[1],aet[2],aet[3]);
								break;
							*/
							case 0x00:
							default:
								printf("Generic error with data: 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X.\n", inPacketBuffer[1], inPacketBuffer[2], inPacketBuffer[3], inPacketBuffer[4], inPacketBuffer[5], inPacketBuffer[6], inPacketBuffer[7], inPacketBuffer[8]);
								break;
						}
						break;
					default:
						//For now just return the packet
						printf("Received Uprocessed Packet: ");
						printf("0x%02X ", headerByte);
						for(uint8_t i=0; i<inPacketBufSize; i++){
							printf("0x%02X ", inPacketBuffer[i]);
						}
						printf("\n");
						break;
				}
				
			}
			inPacketState = 0;
			break;
		default:
			inPacketState = 0;
			break;
	}
}


/*----------------------------------------------------------------
 * main -- execute terminal
 * 
 * Mods: Himanshu Shah, Mark Röling
 * Date: 03/05/18
 *----------------------------------------------------------------
 */


int main(int argc, char **argv)
{
	struct timeb time_buffer;
	char	c;
	char c2;
	time_t start_time, end_time, keep_alive_previous, keep_alive_current;

    

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");
	
	init_js();
	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");

	/* discard any incoming text
	 */
	sleep(1);
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);



	ftime(&time_buffer);
	start_time=time_buffer.time*1000 + time_buffer.millitm;
	end_time = start_time;
	keep_alive_previous = start_time;
	keep_alive_current = start_time;
	term_puts("Comp: Ready.\n");
	


	// Joystick timing test.
    /*
    struct timeb start, end, delta;
    int i = 0;
    ftime(&start);

    while(i++ < 1000) {
        send_j_packet();
    }

    ftime(&end);
    diff = (int) (1000.0 * (end.time - start.time) + (end.millitm - start.millitm));
    printf("\nComputer: Joystick operations took %u milliseconds\n", diff);
    */


 //    //Ping timing test.
 //    struct timeb start, end, delta;
 //    sleep(2);
 //    bool response_found = false;
 //    printf("Computer: Start pings.\n");
 //    ftime(&start);
 //    uint32_t i = 0;
 //    for(i=0; i<1000; i++){
	// 	rs232_putchar(PING);
	// 	while(!response_found){
	// 		if((c = rs232_getchar_nb()) != -1){
	// 			//printf("%c", c);
	// 			if(c == PING){
	// 				response_found = true;
	// 			}
	// 		}
	// 	}
	// 	response_found = false;
	// }
	// ftime(&end);
	// printf("Computer: End pings.\n");
 //    int diff = (int) (1000.0 * (end.time - start.time) + (end.millitm - start.millitm));
 //    printf("\nComputer: %d pings took %u milliseconds. Milliseconds per ping: %.2f\n", i, diff, (float)((float)diff/i));
    

	// Ping test with data and CRC.
	/*
	struct timeb start, end, delta;
	uint8_t data_temp[8];
	printf("Computer: Start ping with data.\n");
	struct packet p_obj;
	p_obj.header=PING_DATCRC;
	p_obj.data=PING_DATCRC;
	p_obj.crc8 = make_crc8_tabled(p_obj.header, &p_obj.data, 1);
	rs232_putchar(p_obj.header);
	rs232_putchar(p_obj.data);
	rs232_putchar(p_obj.crc8);
	while(!response_found){
		if((c = rs232_getchar_nb()) != -1){
			for(uint8_t i=1; i<8; i++){
				data_temp[i] = i-1;
			}
		}
	}
	term_puts("Computer: End ping with data.\n");
	*/


	// sleep(100);
	/* send & receive
	 */
	for (;;)
	{
		read_js_values();
		if((start_time + (1000/JOYSTICK_HZ)) >= end_time){

			if ((c = term_getchar_nb()) != -1){
				//printf("Character found: %c\n", c);
				//rs232_putchar(c);

				if((int)c == 27){							 //detect for escape button and arrowkeys, as arrow keys contains escape character in them
					if((c2 = term_getchar_nb()) == -1){
						panic_now();
						term_puts("Comp: Escape found.\n");
						sleep(1);
						break;
					}
				}else{
					detect_term_input(c);
					//break;
				}
			}else{
				ftime(&time_buffer);
				keep_alive_current=time_buffer.time*1000 + time_buffer.millitm;
				if(keep_alive_current > (keep_alive_previous + KEEP_ALIVE_TIMEOUT_MS)){ // 1 second keepalive
					keep_alive_previous = keep_alive_current;
					struct packet p_obj;
					p_obj.header=PING_DATCRC;
					p_obj.data=PING_DATCRC;
					p_obj.crc8 = make_crc8_tabled(p_obj.header, &p_obj.data, 1);
					rs232_putchar(p_obj.header);
					rs232_putchar(p_obj.data);
					rs232_putchar(p_obj.crc8);

					// TODO: maybe integrate no-ping response message?
				}
				//term_puts("Nothing should be found really...\n");
			}

			if ((c = rs232_getchar_nb()) != -1){
				process_packet(c);
				#if PC_TERMINAL_DISPLAY_INTERFACE_CHARS == 1
				term_putchar(c);
				#endif
			}
			
			//send_j_packet();
		
			ftime(&time_buffer);
			end_time=time_buffer.time*1000 + time_buffer.millitm;
		
		}else{
			ftime(&time_buffer);
			end_time = start_time;
			start_time=time_buffer.time*1000 + time_buffer.millitm;
			keep_alive_previous = start_time; // update the keepalive
			send_j_packet();
		}
		//usleep(10000)
	}
        

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}

