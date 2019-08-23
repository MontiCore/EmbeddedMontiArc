# (c) https://github.com/MontiCore/monticore  
CC=gcc
CFLAGS=-I.

sample_elf: base.o
	$(CC) -Wall -fPIC -c *.cpp
	$(CC) -shared -o sample_syscall.so *.o
