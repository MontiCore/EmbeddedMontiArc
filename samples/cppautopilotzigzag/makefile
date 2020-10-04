CC=g++
CFLAGS=-I.

build: autopilot.o dynamic_interface.o json.o printf.o
#	$(CC) -Wall -fPIC -std=c++11 -c *.cpp
#	$(CC) -shared -std=c++11 -o cppautopilotzigzag.so *.o
	$(CC) -Wall -fPIC -std=c++11 -O3 -static-libgcc -static-libstdc++ -c *.cpp
	$(CC) -shared -std=c++11 -lstdc++ -static-libgcc -static-libstdc++ -Wl,--no-undefined -o cppautopilotzigzag.so *.o