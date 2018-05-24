/*------------------------------------------------------------------
 * logreader.c
 * Log function to read the embedded flash log to a file on the PC
 * 
 * Author: Niels Versluis - 4227646
 *----------------------------------------------------------------*/

#include "in4073.h"
#define LOG_FILENAME "log.txt"

bool read_log_to_file(){
    // Create log file. Overwrite if already exists
    FILE *f = fopen(LOG_FILENAME, "w");
    if(f == NULL){
        printf("ERROR: Could not create log file");      
        return false;
    }
    uint32_t addr = 0x000000;
    bool read_ok = true;
    // Read log data from flash per entry
    while(((addr + LOG_ENTRY_SIZE_BYTES) <= FLASH_ADDR_LIMIT) && read_ok){
        // TODO: Make controller output data in a synchronized manner
        int i = 0;
        while(i < LOG_ENTRY_SIZE_BYTES){
            // TODO: Read character sent by controller
            char read_char = 'a';
            if(read_char != -1){
                fprintf(f, "%c", read_char);
                i++;
            }
        }
        // Write EOL
        fprintf(f, "%s","\n");
        // Iterate address
        addr += LOG_ENTRY_SIZE_BYTES;
    }
    if (!read_ok){
        printf("ERROR: Log read failed!\n");
        return false;
    }
    return true;
}