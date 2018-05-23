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
    if(addr + LOG_ENTRY_SIZE_BYTES > FLASH_ADDR_LIMIT){
        printf("ERROR: Address(%lx) + log size(%x) out of bounds(%x)\n", addr, LOG_ENTRY_SIZE_BYTES, FLASH_ADDR_LIMIT);
        return false;
    } else {
        //Construct log array
        uint8_t array[LOG_ENTRY_SIZE_BYTES];
        /* Time */
        uint32_t t_cur = get_time_us();
        array[0] = t_cur & 0xff;
        array[1] = (t_cur >> (8*1)) & 0xff;
        array[2] = (t_cur >> (8*2)) & 0xff;
        array[3] = (t_cur >> (8*3)) & 0xff;
        /* State */
        //array[4] = state;
        array[4] = QuadState;
        /* Motor values */
        array[5] = ae[0] & 0xff;
        array[6] = (ae[0] >> 8) & 0xff;
        array[7] = ae[1] & 0xff;
        array[8] = (ae[1] >> 8) & 0xff;
        array[9] = ae[2] & 0xff;
        array[10] = (ae[2] >> 8) & 0xff;
        array[11] = ae[3] & 0xff;
        array[12] = (ae[3] >> 8) & 0xff;
        /* MPU Data */
        // phi
        array[13] = phi & 0xff;
        array[14] = (phi >> 8) & 0xff;
        // theta
        array[15] = theta & 0xff;
        array[16] = (theta >> 8) & 0xff;
        // psi
        array[17] = psi & 0xff;
        array[18] = (psi >> 8) & 0xff;
        // sp
        array[19] = sp & 0xff;
        array[20] = (sp >> 8) & 0xff;
        // sq
        array[21] = sq & 0xff;
        array[22] = (sq >> 8) & 0xff;
        // sr
        array[23] = sr & 0xff;
        array[24] = (sr >> 8) & 0xff;

        /* Battery voltage */
        array[25] = bat_volt & 0xff;
        array[26] = (bat_volt >> 8) & 0xff;

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
    if(!spi_flash_init()){
        printf("ERROR: SPI Flash initialization failed\n");
        return false;
    }
    // Create data buffer
    uint8_t data_buf[LOG_ENTRY_SIZE_BYTES];
    flash_read_bytes(addr, data_buf, LOG_ENTRY_SIZE_BYTES);
    uint8_t crc = make_crc8_tabled(BIG_PACKET, data_buf, LOG_ENTRY_SIZE_BYTES);
    printf("Entry(%lu): ", addr);

    // Check for valid data
    uint8_t counter = 0;
    for(uint i=0; i < LOG_ENTRY_SIZE_BYTES; i++){
        if(data_buf[i] == 0xFF){
            counter++;
        }
    }
    if(counter == LOG_ENTRY_SIZE_BYTES){
        printf("Empty flash space found. Exitting.");
        return false;
    }


    // Write log entry to PC
    printf("%02X ", BIG_PACKET);
    for(uint i=0; i < LOG_ENTRY_SIZE_BYTES; i++){
        printf("%02X ", data_buf[i]);
    }
    printf("CRC: %02X\n", crc);
    return true;
}

// Initialize the logfile
bool init_log(){
    prev_log_time = 0;
    prev_write_addr = 0x000000;
    if(spi_flash_init()){
        //if(flash_chip_erase()){ // Not needed since spi_flash_init() already calls a chiperase.
            log_init_done = true;
            return log_init_done;
        //} else {
        //    printf("ERROR: Flash erase failed\n");
        //    return false;
        //}
    } else {
        printf("ERROR: SPI Flash initialization failed\n");
        return false;
    }

}
