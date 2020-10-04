CC=gcc
CFLAGS=-I.

sample_elf: autopilot.o dynamic_interface.o json.o printf.o
#	$(CC) -Wall -fPIC -std=c++11 -c *.cpp
#	$(CC) -shared -std=c++11 -o cppautopilotzigzag.so *.o
	$(CC) -Wall -fPIC -std=c++11 -O3 -static-libgcc -static-libstdc++ -c *.cpp
	$(CC) -shared -std=c++11 -static-libgcc -static-libstdc++ -o cppautopilotzigzag.so *.o