#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <time.h>
#include <signal.h>
#include <errno.h>

// ============= GLOBAL CONFIGURATION =============
#define NUM_UARTS       4       // Configure from 1 to 12 ports
#define MAX_UARTS       12      // Maximum supported ports
// ================================================

// Port configuration structure for easy setup
typedef struct {
    const char *port;
    int baudrate;
    int interval_ms;
} port_config_t;

// ============= USER CONFIGURATION =============
// Configure each port here. Only the first NUM_UARTS entries will be used.
port_config_t port_configs[MAX_UARTS] = {
    // Port 0
    {"/dev/ttyUSB0",  460800, 10},
    // Port 1
    {"/dev/ttyUSB1",  460800, 10},
    // Port 2
    {"/dev/ttyUSB2",  460800, 10},
    // Port 3
    {"/dev/ttyUSB3",  460800, 10},
    // Port 4
    {"/dev/ttyUSB4",  460800, 10},
    // Port 5
    {"/dev/ttyUSB5",  460800, 10},
    // Port 6
    {"/dev/ttyUSB6",  460800, 10},
    // Port 7
    {"/dev/ttyUSB7",  460800, 10},
    // Port 8
    {"/dev/ttyUSB8",  460800, 10},
    // Port 9
    {"/dev/ttyUSB9",  460800, 10},
    // Port 10
    {"/dev/ttyUSB10", 460800, 10},
    // Port 11
    {"/dev/ttyUSB11", 460800, 10}
};
// ==============================================

volatile int keep_running = 1;

// Transmission data arrays - one per port
unsigned char tx_data[MAX_UARTS][150];

typedef struct {
    const char *port;
    int baudrate;
    int fd;
    unsigned char *data;
    size_t size;
    int interval_ms;
    int port_index;
} uart_config_t;

void signal_handler(int sig) {
    keep_running = 0;
}

void initialize_data_arrays(void) {
    // Initialize each port's data array with sequential values
    for (int port = 0; port < MAX_UARTS; port++) {
        for (int i = 0; i < 150; i++) {
            tx_data[port][i] = (unsigned char)i;
        }
    }
}

int set_custom_baudrate(int fd, int baudrate) {
    struct serial_struct ss;
    
    if (ioctl(fd, TIOCGSERIAL, &ss) < 0) {
        perror("TIOCGSERIAL");
        return -1;
    }
    
    ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    ss.custom_divisor = (ss.baud_base + (baudrate / 2)) / baudrate;
    
    if (ioctl(fd, TIOCSSERIAL, &ss) < 0) {
        perror("TIOCSSERIAL");
        return -1;
    }
    
    printf("  Custom baud: %d (divisor=%d, base=%d)\n", baudrate, ss.custom_divisor, ss.baud_base);
    return 0;
}

int open_uart(const char *port, int baudrate, int port_index) {
    printf("[Port %d] Opening %s...\n", port_index, port);
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "[Port %d] Failed to open %s: %s (errno=%d)\n", 
                port_index, port, strerror(errno), errno);
        return -1;
    }
    printf("[Port %d]   Opened successfully (fd=%d)\n", port_index, fd);
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "[Port %d] tcgetattr failed: %s\n", port_index, strerror(errno));
        close(fd);
        return -1;
    }
    
    speed_t speed;
    switch(baudrate) {
        case 9600:    speed = B9600; break;
        case 19200:   speed = B19200; break;
        case 38400:   speed = B38400; break;
        case 57600:   speed = B57600; break;
        case 115200:  speed = B115200; break;
        case 230400:  speed = B230400; break;
        case 460800:  speed = B460800; break;
        case 500000:  speed = B500000; break;
        case 921600:  speed = B921600; break;
        default:
            fprintf(stderr, "[Port %d] Unsupported baudrate %d, using 38400\n", 
                    port_index, baudrate);
            speed = B38400;
            break;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[Port %d] tcsetattr failed: %s\n", port_index, strerror(errno));
        close(fd);
        return -1;
    }
    
    // Check if custom baud rate setup is needed (only for non-standard rates)
    if (baudrate != 9600 && baudrate != 19200 && baudrate != 38400 && 
        baudrate != 57600 && baudrate != 115200 && baudrate != 230400 && 
        baudrate != 460800 && baudrate != 500000 && baudrate != 921600) {
        if (set_custom_baudrate(fd, baudrate) < 0) {
            close(fd);
            return -1;
        }
    }
    
    printf("[Port %d]   Configured @ %d baud\n", port_index, baudrate);
    return fd;
}

void* uart_transmit_thread(void *arg) {
    uart_config_t *config = (uart_config_t *)arg;
    
    // Set highest priority (may require root/CAP_SYS_NICE)
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        // Not fatal - continue without real-time priority
        fprintf(stderr, "[Port %d] Warning: Could not set real-time priority\n", 
                config->port_index);
    }
    
    struct timespec next_time, interval;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    
    interval.tv_sec = config->interval_ms / 1000;
    interval.tv_nsec = (config->interval_ms % 1000) * 1000000L;
    
    unsigned long count = 0;
    unsigned long errors = 0;
    
    printf("[Port %d] Starting transmission @ %d baud, %d ms interval\n", 
           config->port_index, config->baudrate, config->interval_ms);
    
    while (keep_running) {
        // Transmit data
        ssize_t written = write(config->fd, config->data, config->size);
        if (written != (ssize_t)config->size) {
            errors++;
            if (written < 0) {
                fprintf(stderr, "[Port %d] write error: %s (errno=%d)\n", 
                        config->port_index, strerror(errno), errno);
                
                // If device disconnected, stop this thread
                if (errno == ENODEV || errno == ENXIO) {
                    fprintf(stderr, "[Port %d] Device disconnected - stopping thread\n", 
                            config->port_index);
                    break;
                }
            } else {
                fprintf(stderr, "[Port %d] partial write (wrote %ld of %zu bytes)\n", 
                        config->port_index, written, config->size);
            }
        }
        count++;
        
        // Calculate next transmission time
        next_time.tv_sec += interval.tv_sec;
        next_time.tv_nsec += interval.tv_nsec;
        if (next_time.tv_nsec >= 1000000000L) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000L;
        }
        
        // Sleep until next transmission
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
    
    printf("[Port %d] Transmitted %lu times (%lu errors)\n", 
           config->port_index, count, errors);
    return NULL;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Allow runtime override of NUM_UARTS via command line
    int num_ports = NUM_UARTS;
    if (argc > 1) {
        num_ports = atoi(argv[1]);
        if (num_ports < 1 || num_ports > MAX_UARTS) {
            fprintf(stderr, "Error: Number of ports must be between 1 and %d\n", MAX_UARTS);
            return 1;
        }
        printf("Using %d ports (specified on command line)\n", num_ports);
    }
    
    printf("=== UART Multi-Port Transmitter ===\n");
    printf("Maximum ports supported: %d\n", MAX_UARTS);
    printf("Active ports: %d\n\n", num_ports);
    
    printf("Port Configuration:\n");
    for (int i = 0; i < num_ports; i++) {
        printf("  Port %d: %s @ %d baud, %d ms interval\n", 
               i, port_configs[i].port, port_configs[i].baudrate, 
               port_configs[i].interval_ms);
    }
    printf("\n");
    
    // Initialize data arrays
    initialize_data_arrays();
    
    // Allocate configuration structures
    uart_config_t *configs = (uart_config_t *)malloc(num_ports * sizeof(uart_config_t));
    if (!configs) {
        fprintf(stderr, "Failed to allocate memory for configurations\n");
        return 1;
    }
    
    // Configure each UART
    for (int i = 0; i < num_ports; i++) {
        configs[i].port = port_configs[i].port;
        configs[i].baudrate = port_configs[i].baudrate;
        configs[i].fd = -1;
        configs[i].data = tx_data[i];
        configs[i].size = 150;
        configs[i].interval_ms = port_configs[i].interval_ms;
        configs[i].port_index = i;
    }
    
    // Open all UARTs
    printf("Opening ports...\n");
    for (int i = 0; i < num_ports; i++) {
        configs[i].fd = open_uart(configs[i].port, configs[i].baudrate, i);
        if (configs[i].fd < 0) {
            fprintf(stderr, "Failed to open port %d (%s)\n", i, configs[i].port);
            // Close previously opened ports
            for (int j = 0; j < i; j++) {
                if (configs[j].fd >= 0) {
                    close(configs[j].fd);
                }
            }
            free(configs);
            return 1;
        }
    }
    
    printf("\nAll %d ports opened successfully\n", num_ports);
    printf("Press Ctrl+C to stop...\n\n");
    
    // Allocate thread array
    pthread_t *threads = (pthread_t *)malloc(num_ports * sizeof(pthread_t));
    if (!threads) {
        fprintf(stderr, "Failed to allocate memory for threads\n");
        for (int i = 0; i < num_ports; i++) {
            if (configs[i].fd >= 0) close(configs[i].fd);
        }
        free(configs);
        return 1;
    }
    
    // Create transmit threads
    for (int i = 0; i < num_ports; i++) {
        if (pthread_create(&threads[i], NULL, uart_transmit_thread, &configs[i]) != 0) {
            fprintf(stderr, "Failed to create thread for port %d\n", i);
            keep_running = 0;
        }
    }
    
    // Wait for all threads to complete
    for (int i = 0; i < num_ports; i++) {
        pthread_join(threads[i], NULL);
    }
    
    // Close all ports
    printf("\nClosing ports...\n");
    for (int i = 0; i < num_ports; i++) {
        if (configs[i].fd >= 0) {
            close(configs[i].fd);
            printf("[Port %d] Closed\n", i);
        }
    }
    
    // Cleanup
    free(threads);
    free(configs);
    
    printf("\nTransmission stopped\n");
    return 0;
}