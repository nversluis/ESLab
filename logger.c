/*------------------------------------------------------------------
 * logger.c
 * Logging functions to write log data to the on-board flash
 * All code is intended to run on the microcontroller
 * 
 * Author: Niels Versluis - 4227646
 *----------------------------------------------------------------*/
#include "in4073.h"

#define LOG_WRITE_DEBUG 0
#define LOG_READ_DEBUG  1

uint32_t prev_log_time;
uint32_t write_addr;						
uint32_t addr_before_overflow;				// Last address written to before overflow occurred	
int8_t curr_flash_block;					// 4KB flash block currently being written to
bool log_init_done;
bool log_err;
bool log_err_change;
bool flash_overflow;						// True if flash overflow has been triggered
bool empty_flash_read;                      // True if empty flash has been read.

// Initialize the logfile
bool log_init(){
	prev_log_time = get_time_us();
    write_addr = 0x000000;
    addr_before_overflow = 0x000000;
    curr_flash_block = -1;
    flash_overflow = false;
    log_err = false;
	log_err_change = false;
    empty_flash_read = false;

    if(spi_flash_init()){
        log_init_done = true;
        return log_init_done;
    } else {
        remote_print(P_LOGSPIFAIL);
        return false;
    }
}

// Write relevant log data to the flash memory at the specified address
// Returns true on success, false on failure
bool log_write(uint32_t addr){
    if(!log_init_done){
        remote_print(P_LOGFLASHINIT);
        return false;
    }
    // If address out of bounds
    if(addr + LOG_ENTRY_SIZE_BYTES > FLASH_ADDR_LIMIT){
        // I don't feel like porting the actual addresses of this print right now, but if nessecary we could.
        remote_print_data(P_LOGBOUNDSERR, sizeof(addr), (uint8_t*)&addr);
        return false;
    } else {
        //Construct log array
        uint8_t array[LOG_ENTRY_SIZE_BYTES];
        /* Time */
        uint32_t t_cur = get_time_us();
        array[0] = t_cur & 0xFF;
        array[1] = (t_cur >> (8*1)) & 0xFF;
        array[2] = (t_cur >> (8*2)) & 0xFF;
        array[3] = (t_cur >> (8*3)) & 0xFF;
        /* State */
        array[4] = QuadState;
        /* Motor values */
        array[5] = ae[0] & 0xFF;
        array[6] = (ae[0] >> 8) & 0xFF;
        array[7] = ae[1] & 0xFF;
        array[8] = (ae[1] >> 8) & 0xFF;
        array[9] = ae[2] & 0xFF;
        array[10] = (ae[2] >> 8) & 0xFF;
        array[11] = ae[3] & 0xFF;
        array[12] = (ae[3] >> 8) & 0xFF;
        /* MPU Data */
        // phi
        array[13] = (phi-phi_o) & 0xFF;
        array[14] = ((phi-phi_o) >> 8) & 0xFF;
        // theta
        array[15] = (theta-theta_o) & 0xFF;
        array[16] = ((theta-theta_o) >> 8) & 0xFF;
        // psi
        array[17] = (psi-psi_o) & 0xFF;
        array[18] = ((psi-psi_o) >> 8) & 0xFF;
        // sp
        array[19] = (sp-sp_o) & 0xFF;
        array[20] = ((sp-sp_o) >> 8) & 0xFF;
        // sq
        array[21] = (sq-sq_o) & 0xFF;
        array[22] = ((sq-sq_o) >> 8) & 0xFF;
        // sr
        array[23] = (sr-sr_o) & 0xFF;
        array[24] = ((sr-sr_o) >> 8) & 0xFF;
        /* Battery voltage */
        array[25] = bat_volt & 0xFF;
        array[26] = (bat_volt >> 8) & 0xFF;

        if(flash_write_bytes(addr, array, LOG_ENTRY_SIZE_BYTES)){
            return true;
        } else {
            remote_print(P_LOGFLASHWRITE);
            return false;
        }
    }

}

// Push data of a single log entry to the PC
// Returns true on success, false on failure
bool log_read_entry(uint32_t addr){
    if(!log_init_done){
        remote_print(P_LOGNOTINIT);
        return false;
    }
    if(addr + LOG_ENTRY_SIZE_BYTES > FLASH_ADDR_LIMIT){
        remote_print_data(P_LOGBOUNDSERR, sizeof(addr), (uint8_t*)&addr);
        return false;
    }
    // Create data buffer
    uint8_t data_buf[LOG_ENTRY_SIZE_BYTES];
    flash_read_bytes(addr, data_buf, LOG_ENTRY_SIZE_BYTES);

    #if LOG_READ_DEBUG
    remote_print_data(P_LOGENTRY, sizeof(addr), (uint8_t*)&addr);
    #endif

    // Check for valid data
    uint8_t counter = 0;
    for(uint8_t i=0; i < LOG_ENTRY_SIZE_BYTES; i++){
        if(data_buf[i] == 0xFF){
            counter++;
        }
    }
    if(counter == LOG_ENTRY_SIZE_BYTES){
        remote_print(P_LOGEMPTYSPACE);
        empty_flash_read = true;
        return true;
    }

    // Send log entry to PC log reader
    struct packet p_obj;
	p_obj.header = LOG_ENTRY;
	p_obj.crc8 = make_crc8_tabled(p_obj.header, (uint8_t*)&data_buf, LOG_ENTRY_SIZE_BYTES);
	uart_put(p_obj.header);
	for(uint8_t i=0; i<LOG_ENTRY_SIZE_BYTES; i++){
        uart_put((uint8_t)data_buf[i]);
	}
	uart_put(p_obj.crc8);

    return true;
}

// Dump all log entries to PC
void log_dump(){
    // Iterate starting from last address that contains data
    int32_t i = write_addr - LOG_ENTRY_SIZE_BYTES;
    // Normal readout
    while(i >= 0){
        if(!log_read_entry(i)){
            remote_print(P_LOGDUMPABORT);
            flash_overflow = false;
            break;
        } else {
            i -= LOG_ENTRY_SIZE_BYTES;
        }
        // Stop logging if empty space detected
        if (empty_flash_read){
            // End of flash reached, stop sending
            return;
        }
    }
    // Overflow readout
    if(flash_overflow){
        // Read back from end of flash to last non-erased address
        i = addr_before_overflow - LOG_ENTRY_SIZE_BYTES;
        uint32_t lower_addr_limit = (curr_flash_block + 1) * 0x1000;
        // Read down to lower address limit
        while(i > lower_addr_limit){
            if(!log_read_entry(i)){
                remote_print(P_LOGDUMPABORT);
                break;
            } else {
                i -= LOG_ENTRY_SIZE_BYTES;
            }
        }			
    }
}

void logger_main(){
    uint32_t cur_time = get_time_us();
    // Only log when flash is initialized, no logging errors have occured, and log period
    // has expired.
    if((log_init_done) && (!log_err) && (cur_time >= prev_log_time + LOG_PERIOD_US)){
        prev_log_time = cur_time;

        log_err = !log_write(write_addr);
        write_addr += LOG_ENTRY_SIZE_BYTES;
        // If there is flash overflow
        if(write_addr + LOG_ENTRY_SIZE_BYTES > FLASH_ADDR_LIMIT){
            // Wrap around
            addr_before_overflow = write_addr - LOG_ENTRY_SIZE_BYTES;
            write_addr = 0;
            if(flash_overflow == false){
                remote_print(P_LOGOVERFLOW);
            }
            flash_overflow = true;
        }
        // Erase flash sector if necessary
        if(flash_overflow){
            uint8_t requested_block = (uint8_t)floor(write_addr / 0x1000);
            if(curr_flash_block != requested_block){
                #if LOG_READ_DEBUG || LOG_WRITE_DEBUG
                    remote_print_data(P_LOGSWITCH, sizeof(requested_block), &requested_block);
                #endif
                if(flash_4k_sector_erase(requested_block)){
                    curr_flash_block = requested_block;
                } else {
                    remote_print(P_LOGERASEFAIL);                   
                }
            }
        }
    } else if (!log_err_change && log_err){
        remote_print(P_LOGABORT);
        log_err_change = true;
    }
}