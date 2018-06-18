CC=gcc
CFLAGS = -g -Wall -export-dynamic -lm
EXEC = ./pc-interface

all:
	$(CC) $(CFLAGS) *.c ../crc.c ../logwriter.c `pkg-config --cflags --libs gtk+-3.0` -o $(EXEC) #-I /usr/include/gtk-3.0/gtk

run: all
	$(EXEC)
	#nice -n -20 $(EXEC)
