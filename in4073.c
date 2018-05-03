/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include <stdbool.h>

#define MAX_PACKET_SIZE 10

uint8_t inPacketState = 0;
uint8_t headerByte = 0x00;
uint8_t totalBytesToRead = 0;
uint8_t inPacketBuffer[MAX_PACKET_SIZE];
uint8_t inPacketBufSize = 0;

/*------------------------------------------------------------------
 * process_packet -- process incoming packets
 * Mark Röling
 *------------------------------------------------------------------
 */
void process_packet(){
	bool CRCIsValid = true; // Set this to default false when CRC is actually implemented
	uint8_t readByte = 0;
	uint8_t headerFound = false;

	if(rx_queue.count){
		readByte = dequeue(&rx_queue);
		switch(inPacketState){
			case 0:
				// Check if it's a header byte
				if(readByte == 'H'){ // For now only just use a testing one 0x77
					headerByte = readByte;
					totalBytesToRead = 8;
					headerFound = true;
				}
				if(readByte == 0x77){ // For now only just use a testing one 0x77
					headerByte = readByte;
					totalBytesToRead = 2;
					headerFound = true;
				}

				if(headerFound == true){
					inPacketBufSize = 0;
					inPacketState = 1;
					printf("Header byte found: %c\n", headerByte);
				}
				break;
			case 1:
				inPacketBuffer[inPacketBufSize++] = readByte;
				printf("Data byte found: %c\n", readByte);
				if(inPacketBufSize >= totalBytesToRead){
					inPacketState = 2;
					printf("Total databytes read: %d\n", inPacketBufSize);
				}else{
					break;
				}
			case 2:
				// Calculate/validate CRC
				if(CRCIsValid == true){
					printf("CRC is valid (unused for now)\n");
					//Use values
					//For now just return the packet
					printf("Packet: ");
					printf("%c", headerByte);
					for(uint8_t i=0; i<inPacketBufSize; i++){
						printf("%c", inPacketBuffer[i]);
					}
					printf("\n");
				}
				inPacketState = 0;
				break;
		}
	}
}

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c)
{
	switch (c)
	{
		case 'q':
			ae[0] += 10;
			break;
		case 'a':
			ae[0] -= 10;
			if (ae[0] < 0) ae[0] = 0;
			break;
		case 'w':
			ae[1] += 10;
			break;
		case 's':
			ae[1] -= 10;
			if (ae[1] < 0) ae[1] = 0;
			break;
		case 'e':
			ae[2] += 10;
			break;
		case 'd':
			ae[2] -= 10;
			if (ae[2] < 0) ae[2] = 0;
			break;
		case 'r':
			ae[3] += 10;
			break;
		case 'f':
			ae[3] -= 10;
			if (ae[3] < 0) ae[3] = 0;
			break;
		case 27:
			demo_done = true;
			break;
		default:
			nrf_gpio_pin_toggle(RED);
	}
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */
int main(void)
{
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);	
	baro_init();
	spi_flash_init();
	//ble_init();



//	uint32_t counter = 0;
//	demo_done = false;
//
	while (!demo_done)
	{
		process_packet();
//		if (rx_queue.count) process_key( dequeue(&rx_queue) );
//
//		if (check_timer_flag()) 
//		{
//			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);
//
//			adc_request_sample();
//			read_baro();
//
//			printf("%10ld | ", get_time_us());
//			printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
//			printf("%6d %6d %6d | ", phi, theta, psi);
//			printf("%6d %6d %6d | ", sp, sq, sr);
//			printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);
//
//			clear_timer_flag();
//		}
//
//		if (check_sensor_int_flag()) 
//		{
//			get_dmp_data();
//			run_filters_and_control();
//		}
	}	

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
