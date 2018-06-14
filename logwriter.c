/*------------------------------------------------------------------
 * logreader.c
 * Log function to read the embedded flash log to a file on the PC
 * This code is intended to run on a PC
 * All functions return true on success, and false on failure
 * 
 * Author: Niels Versluis - 4227646
 *----------------------------------------------------------------*/

#include "logwriter.h"

// Creates a new, empty logfile
bool init_log_file() {
	// Create log file. Overwrite if exists.
	FILE *f = fopen(LOG_FILENAME, "w");
	if (f == NULL) {
		printf("ERROR: Could not create log file");
		return false;
	}
	fclose(f);
	return true;
}

// Appends a single log entry to the logfile
bool write_log_entry_to_file(uint8_t log_entry_buf[LOG_ENTRY_SIZE_BYTES]) {
	// Open log file. Create if not yet exists
	FILE *f = fopen(LOG_FILENAME, "a+");
	if (f == NULL) {
		printf("ERROR: Could not open log file");
		return false;
	}

	// Initialize variables to be read out of the array
	uint8_t log_state;
	uint16_t log_ae0, log_ae1, log_ae2, log_ae3, log_phi, log_theta,
		log_psi, log_sp, log_sq, log_sr, log_bat_volt;
	uint32_t log_time;
	log_state = log_ae0 = log_ae1 = log_ae2 = log_ae3 = log_phi = log_theta =
		log_psi = log_sp = log_sq = log_sr = log_bat_volt = log_time = 0;
	char log_state_str[16];
	// Convert raw values to useful text
	for (int i = 0; i < LOG_ENTRY_SIZE_BYTES; i++) {
		switch (i) {
			case 0:
				log_time = log_entry_buf[i];
				break;
			case 1:
				log_time |= log_entry_buf[i] << 8;
				break;
			case 2:
				log_time |= log_entry_buf[i] << 16;
				break;
			case 3:
				log_time |= log_entry_buf[i] << 24;
				fprintf(f, "Time: %lu, ", log_time);
				break;
			case 4:
				log_state = log_entry_buf[i];
				switch (log_state) {
					case SAFE:
						strcpy(log_state_str, "SAFE");
						break;
					case PANIC:
						strcpy(log_state_str, "PANIC");
						break;
					case MANUAL:
						strcpy(log_state_str, "MANUAL");
						break;
					case CALIBRATION:
						strcpy(log_state_str, "CALIBRATION");
						break;
					case YAWCONTROL:
						strcpy(log_state_str, "YAWCONTROL");
						break;
					case FULLCONTROL:
						strcpy(log_state_str, "FULLCONTROL");
						break;
					case RAW:
						strcpy(log_state_str, "RAW");
						break;
					case HEIGHT:
						strcpy(log_state_str, "HEIGHT");
						break;
					case WIRELESS:
						strcpy(log_state_str, "WIRELESS");
						break;
					case DUMPLOGS:
						strcpy(log_state_str, "DUMPLOGS");
						break;
				}
				fprintf(f, "state: %s, ", log_state_str);
				break;
			case 5:
				log_ae0 = log_entry_buf[i];
				break;
			case 6:
				log_ae0 |= log_entry_buf[i] << 8;
				fprintf(f, "ae0: %i, ", log_ae0);
				break;
			case 7:
				log_ae1 = log_entry_buf[i];
				break;
			case 8:
				log_ae1 |= log_entry_buf[i] << 8;
				fprintf(f, "ae1: %i, ", log_ae1);
				break;
			case 9:
				log_ae2 = log_entry_buf[i];
				break;
			case 10:
				log_ae2 |= log_entry_buf[i] << 8;
				fprintf(f, "ae2: %i, ", log_ae2);
				break;
			case 11:
				log_ae3 = log_entry_buf[i];
				break;
			case 12:
				log_ae3 |= log_entry_buf[i] << 8;
				fprintf(f, "ae3: %i, ", log_ae3);
				break;
			case 13:
				log_phi = log_entry_buf[i];
				break;
			case 14:
				log_phi |= log_entry_buf[i] << 8;
				fprintf(f, "phi: %i, ", log_phi);
				break;
			case 15:
				log_theta = log_entry_buf[i];
				break;
			case 16:
				log_theta |= log_entry_buf[i] << 8;
				fprintf(f, "theta: %i, ", log_theta);
				break;
			case 17:
				log_psi = log_entry_buf[i];
				break;
			case 18:
				log_psi |= log_entry_buf[i] << 8;
				fprintf(f, "psi: %i, ", log_psi);
				break;
			case 19:
				log_sp = log_entry_buf[i];
				break;
			case 20:
				log_sp |= log_entry_buf[i] << 8;
				fprintf(f, "sp: %i, ", log_sp);
				break;
			case 21:
				log_sq = log_entry_buf[i];
				break;
			case 22:
				log_sq |= log_entry_buf[i] << 8;
				fprintf(f, "sq: %i, ", log_sq);
				break;
			case 23:
				log_sr = log_entry_buf[i];
				break;
			case 24:
				log_sr |= log_entry_buf[i] << 8;
				fprintf(f, "sr: %i, ", log_sr);
				break;
			case 25:
				log_bat_volt = log_entry_buf[i];
				break;
			case 26:
				log_bat_volt |= log_entry_buf[i] << 8;
				fprintf(f, "battery: %u.\n", log_bat_volt);
				break;
		}
	}
	// Close file
	fclose(f);
	return true;
}