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
    if(f == NULL){
        printf("INFO: Could not open log file, creating new file with name %s\n", LOG_FILENAME);
        f = fopen(LOG_FILENAME, "wb");
        if(f == NULL){
            printf("ERROR: Could not create new log file\n");
            return false;
        }
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