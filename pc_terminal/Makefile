CC=gcc
CFLAGS = -g -Wall -lm
EXEC = ./pc-terminal

all:
	$(CC) $(CFLAGS) *.c ../crc.c -o $(EXEC) -lbluetooth

run: all
	$(EXEC)
	#nice -n -20 $(EXEC)
