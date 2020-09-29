CC=gcc
CFLAGS=-I.

sample_elf: autopilot.o dynamic_interface.o json.o printf.o
	$(CC) -Wall -fPIC -std=c++11 -O3 -c *.cpp
	$(CC) -shared -std=c++11 -o cppautopilotzigzag.so *.o
