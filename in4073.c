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
#include <sys/time.h>
#include "pc_terminal/protocol.h"
#include "crc.h"

#define MAX_PACKET_SIZE 10
#define BATTERY_CONNECTED 0
#define PACKET_DEBUG 0

void convert_to_rpm(uint8_t lift, int8_t roll, int8_t pitch, int8_t yaw);

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

//Todo: change the headerFound to a goto to reduce cpu cycles
void process_packet(){
	bool CRCIsValid = true; // Set this to default false when CRC is actually implemented
	uint8_t readByte = 0;
	uint8_t headerFound = false;
	uint8_t crc_calc = 0;
	uint8_t dat_temp = 0;


	if(rx_queue.count > 0){
		readByte = dequeue(&rx_queue);
		//printf("Readbyte: 0x%02X, car is 0x%02X\n", readByte, PING);
		if(readByte == PING){
			uart_put(PING);
			//printf("Pin received.\n");
			inPacketState = 0;
		}
		
		switch(inPacketState){
			case 0:
				if(readByte == PING){
					uart_put(PING);
					//printf("Pin received.\n");
					inPacketState = 0;
					break;
				}
				// Check if it's a header byte
				if(readByte == MODESET || readByte == MODEGET || readByte == K_ROLL || readByte == K_LIFT || readByte == K_YAW || readByte == K_P || readByte == K_P1 || readByte == K_P2 || readByte == K_PITCH || readByte == PING_DATCRC){
					// 1 Byte packets
					headerByte = readByte;
					totalBytesToRead = 2;
					headerFound = true;
				}
				if(readByte == BAT){
					// 2 Byte packets
					headerByte = readByte;
					totalBytesToRead = 3;
					headerFound = true;
				}
				if(readByte == J_CONTROL || readByte == SYSTIME || readByte == PRESSURE){
					// 4 Byte packets
					headerByte = readByte;
					totalBytesToRead = 5;
					headerFound = true;
				}
				if(readByte == J_CONTROL_D || readByte == AE_OUT || readByte == GYRO_OUT || readByte == CAL_GET){
					// 8 Byte packets
					headerByte = readByte;
					totalBytesToRead = 9;
					headerFound = true;
				}

				if(headerFound == true){
					inPacketBufSize = 0;
					inPacketState = 1;
					#if PACKET_DEBUG == 1
					printf("Header byte found: 0x%02X\n", headerByte);
					#endif
				}else{
					#if PACKET_DEBUG == 1
					printf("Non-header found:  0x%02X\n", readByte);
					#endif
				}
				break;
			case 1:
				inPacketBuffer[inPacketBufSize++] = readByte;
				
				if(inPacketBufSize >= totalBytesToRead){
					inPacketState = 2;
					#if PACKET_DEBUG == 1
					printf("CRC byte found:    0x%02X ", readByte);
					#endif
				}else{
					#if PACKET_DEBUG == 1
					printf("Data byte found:   0x%02X\n", readByte);
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
				printf("Total data bytes read: %d\n", inPacketBufSize-1);
				#endif
				if(CRCIsValid == true){
					switch(headerByte){
						case MODESET:
							//printf("Modeset: 0x%02X\n", inPacketBuffer[0]);
							PreviousMode = QuadState;
							ModeToSet = inPacketBuffer[0];
							QuadState = SETNEWMODE;
						case J_CONTROL:
							//printf("Lift: %d, Roll: %d, Pitch: %d, Yaw: %d\n", (uint8_t)inPacketBuffer[0], (int8_t)inPacketBuffer[1], (int8_t)inPacketBuffer[2], (int8_t)inPacketBuffer[3]);
							LRPY[0] = (uint8_t)inPacketBuffer[0];
							LRPY[1] = (int8_t)inPacketBuffer[1];
							LRPY[2] = (int8_t)inPacketBuffer[2];
							LRPY[3] = (int8_t)inPacketBuffer[3];
							break;
						case K_LIFT:
							k_LRPY[0]+=(int8_t)inPacketBuffer[0];
							break;
						case K_ROLL:
							k_LRPY[1]+=(int8_t)inPacketBuffer[0];
							break;
						case K_PITCH:
							k_LRPY[2]+=(int8_t)inPacketBuffer[0];
							break;
						case K_YAW:
							k_LRPY[3]+=(int8_t)inPacketBuffer[0];
							break;
						case K_P:
							k_LRPY[4]+=(int8_t)inPacketBuffer[0];
							break;
						case K_P1:
							k_LRPY[5]+=(int8_t)inPacketBuffer[0];
							break;
						case K_P2:
							k_LRPY[6]+=(int8_t)inPacketBuffer[0];
							break;
						case PING_DATCRC:
							dat_temp = PING_DATCRC;
							printf("%c%c%c\n", PING_DATCRC, dat_temp, make_crc8_tabled(PING_DATCRC, &dat_temp, 1));
						default:
							//For now just return the packet
							printf("Packet: ");
							printf("0x%02X ", headerByte);
							for(uint8_t i=0; i<inPacketBufSize; i++){
								printf("0x%02X ", inPacketBuffer[i]);
							}
							printf("\n");
							break;
					}
					
				}
				inPacketState = 0;
				//nrf_delay_ms(100);
				break;
			default:
				inPacketState = 0;
				break;
		}
	}
}



/*-----------------------------------------------------------------------------------------
* check_battery() -	function to check battery voltage and if voltage is low move to panic
* 						mode.
*
* Author: Himanshu Shah
* Date : 23/05/18
*------------------------------------------------------------------------------------------
*/

void check_battery(){
	// adc_request_sample();
	if((QuadState == SAFE && QuadState == SAFE_NONZERO) && (bat_volt <= 1050)) {
		printf("Battery Critically low (%d volts)!!\n",bat_volt);
		low_battery=true;
	}
	else if(bat_volt > 1050 && bat_volt <=1100){
		printf("Caution!! Battery voltage low!!");
		low_battery=false;
	}
	else if(bat_volt > 1100){
		//printf("Battery voltage %d volts.\n", bat_volt);
		low_battery=false;
	}
	else{
		printf("Battery critically low(%d volts)!! Pleae change the battery and restart......\n",bat_volt);
		low_battery=true; 
		nrf_gpio_pin_toggle(RED);
		QuadState=PANIC;
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
	QuadState = SAFE;
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);	
	baro_init();
	spi_flash_init();
	adc_request_sample();
	//nrf_delay_ms(2000); // Wait 2 seconds for the chip erase to finish
	//ble_init();
	log_init_done = init_log();
	log_err = false;
	bool log_err_change = false;

	prev_log_time = get_time_us();

	uint32_t counter = 0;
	demo_done = false;
	low_battery=false;
	if (BATTERY_CONNECTED){
		check_battery();
	}
	while (!demo_done && !low_battery)
	{
		// if(rx_queue.count > 0){
		// 	uint8_t readByte = 0;
		// 	readByte = dequeue(&rx_queue);
		// 	if(readByte == PING){
		// 		uart_put(PING);
		// 	}
		// }

		process_packet();
//		if (rx_queue.count) process_key( dequeue(&rx_queue) );
//
 		if (check_timer_flag()) 
		{
			
			if (counter++%20 == 0){
			// 	nrf_gpio_pin_toggle(BLUE);

				/*-------------------------------------------------------------------------------------
				* Logger call
				* Author: Niels Versluis
				*------------------------------------------------------------------------------------*/
				uint32_t cur_time = get_time_us();
				// Only log when flash is initialized, no logging errors have occured, and log period
				// has expired.
				if((log_init_done) && (!log_err) && (cur_time >= prev_log_time + LOG_PERIOD_US)){
					prev_log_time = cur_time;
					log_err = !write_log(write_addr);
					write_addr += LOG_ENTRY_SIZE_BYTES;
					// If there is flash overflow
					if(write_addr > FLASH_ADDR_LIMIT){
						// Wrap around
						addr_before_overflow = write_addr;
						write_addr = 0;
						flash_overflow = true;
						printf("WARNING: Flash overflow detected: old data will be erased!");
					}
					// Erase flash sector if necessary
					if(flash_overflow){
						uint8_t requested_block = (uint8_t)floor(write_addr / 4000);
						if(curr_flash_block != requested_block){
							if(flash_4k_sector_erase(requested_block)){
								curr_flash_block = requested_block;
							} else {
								printf("ERROR: 4k Sector erase failed.\n");
							}
						}
					}
				} else if (!log_err_change && log_err){
					printf("ERROR: Logging aborted!\n");
					log_err_change = true;
				}
			}

			if(counter%200 == 0){
				adc_request_sample();
			}

			if(counter%200 == 20){
				if (BATTERY_CONNECTED){
					check_battery();
				}
			}

			run_control();

			// adc_request_sample();
			// read_baro();
//
			// printf("%10ld | ", get_time_us());
			// printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			// printf("%6d %6d %6d | ", phi, theta, psi);
			// printf("%6d %6d %6d | ", sp, sq, sr);
			// printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);
//
 			clear_timer_flag();
 		}
//
		
		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
			run_filters();
		}
		
	}	

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
