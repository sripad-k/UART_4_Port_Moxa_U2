# UART Multi-Port Transmitter (1-12 Ports)

## Overview
Scalable multi-threaded UART transmitter supporting 1 to 12 serial ports simultaneously.
Each port operates independently with configurable baud rate and transmission interval.

## Features
- **Scalable**: Configure from 1 to 12 ports
- **Flexible Configuration**: Each port can have different baud rate and interval
- **Runtime Override**: Specify number of ports via command line
- **Individual Port Control**: Each port runs in its own thread
- **Error Handling**: Per-port error reporting with errno details
- **Real-time Scheduling**: Optional SCHED_FIFO for precise timing

## Quick Start

### 1. Edit Configuration (in uart_transmitter_moxa.c)

```c
#define NUM_UARTS       4       // Default: 1-12 ports

port_config_t port_configs[MAX_UARTS] = {
    // {port_path,        baudrate, interval_ms}
    {"/dev/ttyUSB0",     460800,   10},  // Port 0
    {"/dev/ttyUSB1",     460800,   10},  // Port 1
    {"/dev/ttyUSB2",     115200,   50},  // Port 2 - different settings
    {"/dev/ttyUSB3",     460800,   10},  // Port 3
    // ... up to 12 ports
};
```

### 2. Compile

```bash
make clean
make
```

### 3. Run

```bash
# Use default NUM_UARTS (4 ports)
./uart_transmitter

# Override to use 2 ports
./uart_transmitter 2

# Override to use 8 ports
./uart_transmitter 8

# With real-time priority (requires root)
sudo ./uart_transmitter

```

## Output Example

```

❯ sudo ./uart_transmitter 4
Using 4 ports (specified on command line)
=== UART Multi-Port Transmitter ===
Maximum ports supported: 12
Active ports: 4

Port Configuration:
  Port 0: /dev/ttyUSB0 @ 460800 baud, 10 ms interval
  Port 1: /dev/ttyUSB1 @ 460800 baud, 10 ms interval
  Port 2: /dev/ttyUSB2 @ 460800 baud, 10 ms interval
  Port 3: /dev/ttyUSB3 @ 460800 baud, 10 ms interval

Opening ports...
[Port 0] Opening /dev/ttyUSB0...
[Port 0]   Opened successfully (fd=3)
[Port 0]   Configured @ 460800 baud
[Port 1] Opening /dev/ttyUSB1...
[Port 1]   Opened successfully (fd=4)
[Port 1]   Configured @ 460800 baud
[Port 2] Opening /dev/ttyUSB2...
[Port 2]   Opened successfully (fd=5)
[Port 2]   Configured @ 460800 baud
[Port 3] Opening /dev/ttyUSB3...
[Port 3]   Opened successfully (fd=6)
[Port 3]   Configured @ 460800 baud

All 4 ports opened successfully
Press Ctrl+C to stop...

[Port 0] Starting transmission @ 460800 baud, 10 ms interval
[Port 1] Starting transmission @ 460800 baud, 10 ms interval
[Port 3] Starting transmission @ 460800 baud, 10 ms interval
[Port 2] Starting transmission @ 460800 baud, 10 ms interval
^C[Port 0] Transmitted 116 times (0 errors)
[Port 1] Transmitted 116 times (0 errors)
[Port 3] Transmitted 116 times (0 errors)
[Port 2] Transmitted 116 times (0 errors)

Closing ports...
[Port 0] Closed
[Port 1] Closed
[Port 2] Closed
[Port 3] Closed

Transmission stopped
