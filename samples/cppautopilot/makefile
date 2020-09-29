CC=gcc
CFLAGS=-I.

sample_elf: autopilot.o dynamic_interface.o
	$(CC) -Wall -fPIC -std=c++11 -c *.cpp
	$(CC) -shared -std=c++11 -o cppautopilot.so *.o
