/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <conio.h>
#include <windows.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include "protocol.h"
#include "joystick.h"

HANDLE hSerial;

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */

char term_getchar()
{
	if (_kbhit())
	{
		return _getch();
	}

	return -1;
}

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */


void rs232_open()
{

	DCB dcbSerialParams = { 0 };
	
	//Open Serial port in blocking mode
	hSerial = CreateFile(
		"\\\\.\\COM10", GENERIC_READ | GENERIC_WRITE, 0, NULL,
		OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (hSerial == INVALID_HANDLE_VALUE)
	{
		printf("\r\nCOM port cannot be opened\r\n");
		exit(1);
	}

	PurgeComm(hSerial, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (GetCommState(hSerial, &dcbSerialParams) == 0)
	{
		printf("\r\nError reading COM properties\r\n");
		CloseHandle(hSerial);
		exit(1);
	}

	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (SetCommState(hSerial, &dcbSerialParams) == 0)
	{
		printf("\r\nError setting COM properties\r\n");
		CloseHandle(hSerial);
		exit(1);
	}
}


void 	rs232_close(void)
{
	CloseHandle(hSerial);
}


char rs232_getchar()
{
	char data[1];
	DWORD bytes_read;
	while (!ReadFile(hSerial, data, 1, &bytes_read, NULL))
	{
	}
	return data[0];
}

void rs232_putchar(char c)
{
	char data[1];
	DWORD bytes_written = 0;

	data[0] = c;
	if (!WriteFile(hSerial, data, 1, &bytes_written, NULL))
	{
		printf("\r\nError writing data to COM port\r\n");
	}
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
	char c;
	char c2;
	time_t start_time, end_time;

	term_puts("\nTerminal program - Embedded Real-Time Systems Group 18\n");
	
	init_js();
	rs232_open();

	/* discard any incoming text
	 */
	while ((c = rs232_getchar()) != -1)
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
				}
				else{
					detect_term_input(c);
					break;
				}
			}

			if ((c = rs232_getchar()) != -1)
				printf("%c",c);
        	
			//send_j_packet();
		
			ftime(&time_buffer);
			end_time=time_buffer.time*1000 + time_buffer.millitm;
		
		}

		else{
			ftime(&time_buffer);
			start_time=time_buffer.time*1000 + time_buffer.millitm;
			end_time = start_time;
			send_j_packet();
		}
		//usleep(10000)
	}
        

	rs232_close();
	printf("\r\n<exit>\r\n");

	return 0;
}

