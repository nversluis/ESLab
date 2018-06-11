#!/bin/bash
gcc -o gladewin gui.c -Wall `pkg-config --cflags --libs gtk+-3.0` -export-dynamic