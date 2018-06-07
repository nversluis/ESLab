/*------------------------------------------------------------------
 * logreader.c
 * Log function to read the embedded flash log to a file on the PC
 * This code is intended to run on a PC
 * 
 * Author: Niels Versluis - 4227646
 *----------------------------------------------------------------*/

#include "in4073.h"
#define LOG_FILENAME "log.txt"

#define INIT_ST     0x0
#define READ_ST     0x1

#define LOG_ERR     0xA

uint8_t read_log_entry(uint8_t *buffer){
    uint8_t reader_state = INIT_ST;
    uint8_t log_entry[LOG_ENTRY_SIZE_BYTES] = {0};
    uint8_t header = 0;
    uint8_t crc_packet = 0;
    uint8_t crc_check = 0;
    int i;
    switch(reader_state){
        case INIT_ST:  
            // Keep looking for header packet
            while(header == 0){  
                if(rx_queue.count > 0){
                    header = dequeue(&rx_queue);
                    switch(header){
                        case LOG_ENTRY:
                            reader_state = READ_ST;
                            break;
                        case LOG_START:
                            return LOG_START;
                        case LOG_END:
                            return LOG_END;
                        default:
                            printf("WARNING: Received non-log packet while reading log: %x", header);
                    } 
                }
            }
            break;
            // Read rest of the data
        case READ_ST:
            i = 0;
            while(i < LOG_ENTRY_SIZE_BYTES){
                if(rx_queue.count > 0){
                    log_entry[i] = dequeue(&rx_queue);
                    i++;
                }
            }
            break;
    }
    // Obtain CRC packet
    while(crc_packet == 0)
    {
        if(rx_queue.count > 0){
            crc_packet = dequeue(&rx_queue);
        }
    }
    // Validate CRC
    crc_check = make_crc8_tabled(header, log_entry, LOG_ENTRY_SIZE_BYTES);
    if(crc_packet == crc_check){
        return LOG_ENTRY;
    } else {
        printf("WARNING: Log entry CRC mismatch");
        return LOG_ERR;
    }
}

bool read_log_to_file(){
    // Create log file. Overwrite if already exists
    FILE *f = fopen(LOG_FILENAME, "w");
    if(f == NULL){
        printf("ERROR: Could not create log file");      
        return false;
    }
    bool log_done = false;
    uint8_t log_entry[LOG_ENTRY_SIZE_BYTES] = {0};
    // Read log data from flash per entry
    while(!log_done){
        uint8_t result = read_log_entry(log_entry);
        // Wait for start header
        while(result != LOG_START){
            result = read_log_entry(log_entry);
        }
        fprintf(f, "%s", "* START OF LOG *\n");
        // Get next entry
        result = read_log_entry(log_entry);
        if(result == LOG_ENTRY){
            // Write entry to file
            for(int i = 0; i < LOG_ENTRY_SIZE_BYTES; i++){
                fprintf(f, "%c", (char)log_entry[i]);
            } 
            // Write EOL
            fprintf(f, "%s","\n");
        } else if (result == LOG_END){
            log_done = true;
            fprintf(f, "%s", "* END OF LOG *\n");
        } else if (result == LOG_ERR){
            fprintf(f, "%s", "* THIS ENTRY FAILED TO TRANSFER *");
        }
    }
    return true;
}