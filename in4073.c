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
#include <sys/time.h>
#include "pc_terminal/protocol.h"
#include "crc.h"

void convert_to_rpm(uint8_t lift, int8_t roll, int8_t pitch, int8_t yaw);

uint8_t inPacketState = 0;
uint8_t headerByte = 0x00;
uint8_t totalBytesToRead = 0;
uint8_t inPacketBuffer[MAX_PACKET_SIZE];
uint8_t inPacketBufSize = 0;

/*------------------------------------------------------------------
 * respond_to_ping -- send a response to a ping back to computer
 * Mark Röling
 *------------------------------------------------------------------
 */
void respond_to_ping(){
    struct packet p_obj;
	p_obj.header=PING_DATACK;
	p_obj.data=PING_DATACK;
	p_obj.crc8 = make_crc8_tabled(p_obj.header, &p_obj.data, 1);
	uart_put(p_obj.header);
	uart_put(p_obj.data);
	uart_put(p_obj.crc8);
}

/*------------------------------------------------------------------
 * send_bat_voltage -- send the battery voltage to computer
 * Mark Röling
 *------------------------------------------------------------------
 */
void send_bat_voltage(){
    struct packet p_obj;
	p_obj.header=BAT;
	p_obj.crc8 = make_crc8_tabled(p_obj.header, (uint8_t*)&bat_volt, sizeof(bat_volt));
	uart_put(p_obj.header);
	uart_put((uint8_t)(bat_volt & 0xFF));
	uart_put((uint8_t)(bat_volt>>8 & 0xFF));
	uart_put(p_obj.crc8);
}

/*------------------------------------------------------------------
 * send_calibration_data -- sends calibration data to comptuer
 * Mark Röling
 *------------------------------------------------------------------
 */
void send_calibration_data(){
    struct packet p_obj;
    int16_t offsets[6] = {phi_o, theta_o, psi_o, sp_o, sq_o, sr_o};
	p_obj.header=CAL_GET;
	p_obj.crc8 = make_crc8_tabled(p_obj.header, (uint8_t*)&offsets, sizeof(offsets));
	uart_put(p_obj.header);
	for(uint8_t i=0; i<6; i++){
		uart_put((uint8_t)(offsets[i] & 0xFF));
		uart_put((uint8_t)(offsets[i]>>8 & 0xFF));
	}
	uart_put(p_obj.crc8);
}

/*------------------------------------------------------------------
 * send_motor_data -- sends motor data to comptuer
 * Mark Röling
 *------------------------------------------------------------------
 */
void send_motor_data(){
    struct packet p_obj;
	p_obj.header=AE_OUT;
	p_obj.crc8 = make_crc8_tabled(p_obj.header, (uint8_t*)&ae, sizeof(ae));
	uart_put(p_obj.header);
	for(uint8_t i=0; i<4; i++){
		uart_put((uint8_t)(ae[i] & 0xFF));
		uart_put((uint8_t)(ae[i]>>8 & 0xFF));
	}
	uart_put(p_obj.crc8);
}

/*------------------------------------------------------------------
 * process_packet -- process incoming packets
 * Mark Röling
 *------------------------------------------------------------------
 */

//Todo: change the headerFound to a goto to reduce cpu cycles
void process_packet(){
	bool CRCIsValid = false;
	uint8_t readByte = 0;
	uint8_t headerFound = false;
	uint8_t crc_calc = 0;
	#if PACKET_DEBUG == 1
	uint8_t dat_temp = 0;
	#endif


	if(rx_queue.count > 0){
		readByte = dequeue(&rx_queue);
		received_data = true;
		//printf("Readbyte: 0x%02X, car is 0x%02X\n", readByte, PING);
		/*
		if(readByte == PING){
			uart_put(PING);
			//printf("Pin received.\n");
			inPacketState = 0;
		}
		*/
		
		switch(inPacketState){
			case 0:
				// if(readByte == PING){
				// 	uart_put(PING);
				// 	//printf("Pin received.\n");
				// 	inPacketState = 0;
				// 	break;
				// }
				// Check if it's a header byte
				if(readByte == MODESET || readByte == MODEGET || readByte == K_ROLL || readByte == K_LIFT || readByte == K_YAW || readByte == K_P || readByte == K_P1 || readByte == K_P2 || readByte == K_HEIGHT || readByte == K_PITCH || readByte == PING_DATCRC){
					// 1 Byte packets
					headerByte = readByte;
					totalBytesToRead = 2;
					headerFound = true;
				}
				else if(readByte == BAT){
					// 2 Byte packets
					headerByte = readByte;
					totalBytesToRead = 3;
					headerFound = true;
				}
				else if(readByte == J_CONTROL || readByte == SYSTIME || readByte == PRESSURE){
					// 4 Byte packets
					headerByte = readByte;
					totalBytesToRead = 5;
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
						case K_HEIGHT:
							k_LRPY[7]+=(int8_t)inPacketBuffer[0];
							break;
						case PING_DATCRC:
							#if PACKET_DEBUG == 1
							dat_temp = PING_DATCRC;
							printf("%c%c%c\n", PING_DATCRC, dat_temp, make_crc8_tabled(PING_DATCRC, &dat_temp, 1));
							#endif
							respond_to_ping();
							break;
						default:
							#if PACKET_DEBUG == 1
							//For now just return the packet
							printf("Packet: ");
							printf("0x%02X ", headerByte);
							for(uint8_t i=0; i<inPacketBufSize; i++){
								printf("0x%02X ", inPacketBuffer[i]);
							}
							printf("\n");
							#endif
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
	send_bat_voltage();
	if((QuadState == SAFE && QuadState == SAFE_NONZERO) && (bat_volt <= 1050)) {
		remote_print_data(P_BATCRIT, sizeof(bat_volt), (uint8_t*)&bat_volt);
		//printf("Battery Critically low (%d volts)!!\n",bat_volt);
		low_battery=true;
	}
	else if(bat_volt > 1050 && bat_volt <=1100){
		remote_print_data(P_BATLOW, sizeof(bat_volt), (uint8_t*)&bat_volt);
		//printf("Caution!! Battery voltage low!!");
		low_battery=false;
	}
	else if(bat_volt > 1100){
		//printf("Battery voltage %d volts.\n", bat_volt);
		low_battery=false;
	}
	else{
		remote_print_data(P_BATCRIT, sizeof(bat_volt), (uint8_t*)&bat_volt);
		//printf("Battery critically low(%d volts)!! Pleae change the battery and restart......\n",bat_volt);
		low_battery=true; 
		nrf_gpio_pin_toggle(RED);
		QuadState=PANIC;
	}
}

/*-----------------------------------------------------------------------------------------
* remote_print() -	functions to print a print statement on remote computer terminal
*
* Author: Mark Röling
* Date : 23/05/18
*------------------------------------------------------------------------------------------
*/

void remote_print(uint8_t printCase){
    struct packet p_obj;
	p_obj.header=PRINT;
	p_obj.data=printCase;
	p_obj.crc8 = make_crc8_tabled(p_obj.header, &p_obj.data, 1);
	uart_put(p_obj.header);
	uart_put(p_obj.data);
	uart_put(p_obj.crc8);
}
void remote_print_data(uint8_t printCase, uint8_t dataBytes, uint8_t* data){
	uint8_t datat[5];
	uint8_t header = 0x00;
	//printf("\nP");
    switch(dataBytes){
    	case 1:
    		//printf("1");
			header = PRINT1;
			break;
    	case 2:
    		//printf("2");
			header = PRINT2;
			break;
   		case 4:
   			//printf("4");
			header = PRINT4;
			break;
		default:
			// Size isn´t supported yet. Maybe throw some form of error?
			return;
			break;
    }
    
    datat[0] = printCase;
    //printf(": 0x%02X ", datat[0]);
    for(uint8_t i=0; i<dataBytes; i++){
    	datat[i+1] = data[i];
    	//printf("0x%02X ", datat[i+1]);
    }

	uint8_t crc = make_crc8_tabled(header, datat, dataBytes+1);
	//printf("CRC: 0x%02X.\n", crc);
	//nrf_delay_ms(1000);
	uart_put(header);
	for(uint8_t i=0; i<dataBytes+1; i++){
		uart_put(datat[i]);
	}
	uart_put(crc);
}
void remote_notify_state(uint8_t state, uint8_t answer){
    struct packet p_obj;
    if(answer == ACK){
		p_obj.header=MODESET;
	}else{
		p_obj.header=MODEGET;
	}
	p_obj.data=state;
	p_obj.crc8 = make_crc8_tabled(p_obj.header, &p_obj.data, 1);
	uart_put(p_obj.header);
	uart_put(p_obj.data);
	uart_put(p_obj.crc8);
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */
int main(void)
{
	QuadState = SAFE;
	received_data = true;
	BlinkLed = false;
	USBDisconnected = false;
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);	
	baro_init();
	spi_flash_init();
	adc_request_sample();
	//ble_init();
	log_init();
	nrf_delay_ms(1100); // Wait 1100ms for the computer program to start reading data.

	uint32_t counter = 0;
	demo_done = false;
	low_battery=false;
	if (BATTERY_CONNECTED){
		check_battery();
	}
	//printf("BLINKLED_CYCLES: %u, USB_TIMEOUT_CYCLES: %u, USB_TIMEOUT_MS: %u, KEEP_ALIVE_TIMEOUT_MS: %u.\n", BLINKLED_CYCLES, USB_TIMEOUT_CYCLES, USB_TIMEOUT_MS, KEEP_ALIVE_TIMEOUT_MS);

	//printf("Entering main loop...\n");
	remote_print(P_MAINLOOP);
	//uint32_t testvar = 0x01020304;
	//remote_print_data(P_TEST4, sizeof(testvar), (uint8_t*)&testvar);

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

 		if (check_timer_flag()) 
		{
			counter++;

			#if LOG_DEBUG
			if (counter%1 == 0){
			#else
			if (counter%20 == 0){
			#endif
				logger_main();
			}

			// Check for battery voltage
			if(counter%200 == 0){
				adc_request_sample();
			}
			if(counter%200 == 20){
				if (BATTERY_CONNECTED){
					check_battery();
				}
			}

			// Check for dead usb connection -- Mark Röling
			if(counter%(USB_TIMEOUT_CYCLES) == 0){
				if(received_data){
					// All is good
					received_data = false;
					BlinkLed = false;
					nrf_gpio_pins_clear(BLUE);
					USBDisconnected = false;
				}else{
					// Shit hit the fan
					QuadState = PANIC;
					BlinkLed = true;
					USBDisconnected = true;
				}
			}

			// Send motor data -- Mark Röling
			if(counter%(MOTORDATA_CYCLES) == 10){
				send_motor_data();
			}

			// Blink led -- Mark Röling
			if(BlinkLed && counter%(BLINKLED_CYCLES) == 0){
				nrf_gpio_pin_toggle(BLUE);
			}

			run_control();

			// adc_request_sample();
			// read_baro();

			// printf("%10ld | ", get_time_us());
			// printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			// printf("%6d %6d %6d | ", phi, theta, psi);
			// printf("%6d %6d %6d | ", sp, sq, sr);
			// printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);=

 			clear_timer_flag();
 		}

		
		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
			run_filters();
		}
		
	}	

	remote_print(P_GOODBYE);
	//printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
