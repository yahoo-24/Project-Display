# Set compiler
CC=g++

# Compile flags (-c to output object file .o)
CFLAGS=-c -Wall

# Default command, use "make" to call this
all: main.exe

# Linking step, .o files are made for each .cpp file
# $@ is the target
main.exe: main.o LoopPath.o funcs.o Control.o Colours.o Robot.o Battery_Items.o
	echo "You are using Makefile, well done!"
	$(CC) main.o LoopPath.o funcs.o Control.o Colours.o Robot.o Battery_Items.o -o $@

main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp

LoopPath.o: LoopPath.cpp
	$(CC) $(CFLAGS) LoopPath.cpp

funcs.o: funcs.cpp
	$(CC) $(CFLAGS) funcs.cpp

Control.o: Control.cpp
	$(CC) $(CFLAGS) Control.cpp

Colours.o: Colours.cpp
	$(CC) $(CFLAGS) Colours.cpp

Robot.o: Robot.cpp
	$(CC) $(CFLAGS) Robot.cpp

Battery_Items.o: Battery_Items.cpp
	$(CC) $(CFLAGS) Battery_Items.cpp

# Use "make clean" to run this. Note: rm doesn't work on Windows
clean:
	rm -f *.o main.exe