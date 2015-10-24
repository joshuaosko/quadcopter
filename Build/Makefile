OBJECTS =  command_processor.o
CFLAGS = -I../../../Libraries/BlackLib/v3.0
CC = g++
TARGETDIR = Bin

all: $(OBJECTS)

command_processor.o : command_processor.cpp
	$(CC) $(CFLAGS) $(TARGETDIR) $@ $?	
