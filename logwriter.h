#ifndef _LOG_LIB_H
#define _LOG_LIB_H

#include "pc_terminal/protocol.h"
#include <stdio.h>
#define LOG_FILENAME "log.txt"

bool write_log_entry_to_file(uint8_t log_entry[]);
bool init_log_file();

#endif