/*------------------------------------------------------------------
 * logreader.c
 * Log function to read the embedded flash log to a file on the PC
 * 
 * Author: Niels Versluis - 4227646
 *----------------------------------------------------------------*/

#include "in4073.h"
#define LOG_FILENAME "log.txt"
bool read_log_to_file(){
    // Open log file to write to
    FILE *f = fopen(LOG_FILENAME, "w");
    if(f = NULL){
        printf("ERROR: Could not open log file");
        return false;
    }
    uint32_t addr = 0x000000;
    bool read_ok = true;
    // Read log data from flash per entry
    while(((addr + LOG_ENTRY_SIZE_BYTES) <= FLASH_ADDR_LIMIT) && read_ok){
        // Make controller output entry to RS232
        // read_ok = read_log_entry(addr);
        // Read entry from RS232
        int i = 0;
        while(i < LOG_ENTRY_SIZE_BYTES){
            char read_char = rs232_getchar_nb();
            if(read_char != -1){
                fprint(f, "%c", read_char);
                i++;
            }
        }
        // Write EOL
        fprintf(f, "%s","\n");
        // Iterate address
        addr += LOG_ENTRY_SIZE_BYTES;
    }
    if (!read_ok){
        printf("ERROR: Log read failed!");
    }
}