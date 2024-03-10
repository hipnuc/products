CC=gcc

# Include directory for header files
INCLUDE_DIRS=../lib

SOURCES=main.c serial_port.c ../lib/hipnuc_dec.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=main

# Add the include directory flag
CFLAGS=-I$(INCLUDE_DIRS)

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $@
	@echo "Cleaning up..."
	@rm -f $(OBJECTS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f $(OBJECTS) $(EXECUTABLE)
