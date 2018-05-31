/*------------------------------------------------------------------
 * logger.c
 * Logging functions to write log data to the on-board flash
 * 
 * Author: Niels Versluis - 4227646
 *----------------------------------------------------------------*/
#include "in4073.h"
#include <stdbool.h>
#include "crc.h"

#define LOG_DUMP_DEBUG 0

// Write relevant log data to the flash memory at the specified address
// Returns true on success, false on failure
bool write_log(uint32_t addr){
    if(!log_init_done){
        printf("ERROR: Flash not initalized yet!");
        return false;
    }
    // If address out of bounds
    if(addr + LOG_ENTRY_SIZE_BYTES > FLASH_ADDR_LIMIT){
        printf("ERROR: Address(%lx) + log size(%x) out of bounds(%x)\n", addr, LOG_ENTRY_SIZE_BYTES, FLASH_ADDR_LIMIT);
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

        #if LOG_DUMP_DEBUG == 1
        printf("Log Written: ");
        for(uint8_t i=0; i<LOG_ENTRY_SIZE_BYTES; i++){
            printf("%02X ", array[i]);
        }
        printf("\n");
        #endif

        if(flash_write_bytes(addr, array, LOG_ENTRY_SIZE_BYTES)){
            return true;
        } else {
            printf("ERROR: Flash write failed\n");
            return false;
        }
    }

}

// Push all log data to the PC
// Returns true on success, false on failure
bool read_log_entry(uint32_t addr){
    if(!log_init_done){
        printf("ERROR: Log not initialized\n");
        return false;
    }
    // Create data buffer
    uint8_t data_buf[LOG_ENTRY_SIZE_BYTES];
    flash_read_bytes(addr, data_buf, LOG_ENTRY_SIZE_BYTES);
    uint8_t crc = make_crc8_tabled(BIG_PACKET, data_buf, LOG_ENTRY_SIZE_BYTES);
    printf("Entry(%lu): ", addr);

    // Check for valid data
    uint8_t counter = 0;
    for(uint8_t i=0; i < LOG_ENTRY_SIZE_BYTES; i++){
        if(data_buf[i] == 0xFF){
            counter++;
        }
    }
    if(counter == LOG_ENTRY_SIZE_BYTES){
        printf("INFO: Empty flash space found.\n");
        return false;
    }


    // Write log entry to PC
    printf("%02X ", BIG_PACKET);
    for(uint8_t i=0; i < LOG_ENTRY_SIZE_BYTES; i++){
        printf("%02X ", data_buf[i]);
        nrf_delay_ms(1);
    }
    nrf_delay_ms(1);
    printf("CRC: %02X\n", crc);
    nrf_delay_ms(1);
    return true;
}

// Initialize the logfile
bool init_log(){
    prev_log_time = 0;
    write_addr = 0x000000;
    addr_before_overflow = 0x000000;
    curr_flash_block = -1;
    flash_overflow = false;
    if(spi_flash_init()){
        log_init_done = true;
        return log_init_done;
    } else {
        printf("ERROR: SPI Flash initialization failed\n");
        return false;
    }

}
