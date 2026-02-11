CC = gcc
CFLAGS = -Wall -O2 -pthread
TARGET = uart_transmitter

all: $(TARGET)

$(TARGET): uart_transmitter_moxa.c
	$(CC) $(CFLAGS) -o $(TARGET) uart_transmitter_moxa.c

clean:
	rm -f $(TARGET)

.PHONY: all clean
