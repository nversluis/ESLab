/*------------------------------------------------------------------
 * logreader.c
 * Log function to read the embedded flash log to a file on the PC
 * This code is intended to run on a PC
 * 
 * Author: Niels Versluis - 4227646
 *----------------------------------------------------------------*/

#include "logwriter.h"

bool write_log_entry_to_file(uint8_t log_entry[]){
    // Open log file. Create if not yet exists
    FILE *f = fopen(LOG_FILENAME, "a+");
    if(f == NULL){
        printf("ERROR: Could not open log file");      
        return false;
    }

    // Write next entry to file
    for(int i = 0; i < LOG_ENTRY_SIZE_BYTES; i++){ 
        fprintf(f, "%c", (char)log_entry[i]);
    } 

    // Write EOL
    fprintf(f, "%s","\n");
    // Close file
    fclose(f);
    return true;
}

bool init_log_file(){
    // Create log file. Overwrite if exists.
    FILE *f = fopen(LOG_FILENAME, "w");
    if(f == NULL){
        printf("ERROR: Could not create log file");      
        return false;
    }
    return true;
}