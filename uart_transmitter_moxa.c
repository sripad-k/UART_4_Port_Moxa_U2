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
#define NUM_UARTS       4       // Moxa UPort 1450-G2 has 4 ports (0-3)
// ================================================

// ============= USER CONFIGURATION =============
// Using Moxa UPort 1450-G2
#define UART1_PORT      "/dev/ttyUSB0"
#define UART1_BAUDRATE  460800
#define UART1_INTERVAL  10      // milliseconds

#define UART2_PORT      "/dev/ttyUSB1"
#define UART2_BAUDRATE  460800
#define UART2_INTERVAL  10      // milliseconds

#define UART3_PORT      "/dev/ttyUSB2"
#define UART3_BAUDRATE  460800
#define UART3_INTERVAL  10      // milliseconds

#define UART4_PORT      "/dev/ttyUSB3"
#define UART4_BAUDRATE  460800
#define UART4_INTERVAL  10      // milliseconds
// ==============================================

volatile int keep_running = 1;

unsigned char data1[150] = { 
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 
    75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 
    125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149};

unsigned char data2[150] = { 
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 
    75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 
    125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149};

unsigned char data3[150] = { 
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 
    75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 
    125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149};

unsigned char data4[150] = { 
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 
    25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 
    75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 
    125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149};

typedef struct {
    const char *port;
    int baudrate;
    int fd;
    unsigned char *data;
    size_t size;
    int interval_ms;
    const char *name;
} uart_config_t;

void signal_handler(int sig) {
    keep_running = 0;
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
    
    printf("Custom baud: %d (divisor=%d, base=%d)\n", baudrate, ss.custom_divisor, ss.baud_base);
    return 0;
}

int open_uart(const char *port, int baudrate) {
    printf("Opening %s...\n", port);
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Failed to open %s: %s (errno=%d)\n", port, strerror(errno), errno);
        return -1;
    }
    printf("  Opened successfully (fd=%d)\n", fd);
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
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
        // case 512000:  speed = B512000; break;
        case 921600:  speed = B921600; break;
        default:
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
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    
    // Check if custom baud rate setup is needed (only for non-standard rates)
    if (baudrate != 9600 && baudrate != 19200 && baudrate != 38400 && 
        baudrate != 57600 && baudrate != 115200 && baudrate != 230400 && 
        baudrate != 460800 && baudrate != 500000 && baudrate != 512000 && 
        baudrate != 921600) {
        if (set_custom_baudrate(fd, baudrate) < 0) {
            close(fd);
            return -1;
        }
    }
    
    return fd;
}

void* uart_transmit_thread(void *arg) {
    uart_config_t *config = (uart_config_t *)arg;
    
    // Set highest priority
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    
    struct timespec next_time, interval;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    
    interval.tv_sec = config->interval_ms / 1000;
    interval.tv_nsec = (config->interval_ms % 1000) * 1000000L;
    
    unsigned long count = 0;
    
    printf("%s: Starting transmission @ %d baud, %d ms interval\n", 
           config->name, config->baudrate, config->interval_ms);
    
    while (keep_running) {
        // Transmit data
        ssize_t written = write(config->fd, config->data, config->size);
        if (written != (ssize_t)config->size) {
            if (written < 0) {
                fprintf(stderr, "%s: write error: %s (errno=%d)\n", 
                        config->name, strerror(errno), errno);
            } else {
                fprintf(stderr, "%s: partial write (wrote %ld of %zu bytes)\n", 
                        config->name, written, config->size);
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
    
    printf("%s: Transmitted %lu times\n", config->name, count);
    return NULL;
}

int main() {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("=== UART Continuous Transmitter ===\n");
    printf("Device: Moxa UPort 1450-G2 (Serial: TBEEB1106908)\n");
    printf("Configuration (%d UARTs):\n", NUM_UARTS);
    
#if NUM_UARTS >= 1
    printf("  Port 0 (ttyUSB0) @ %d baud, %d ms interval\n", UART1_BAUDRATE, UART1_INTERVAL);
#endif
#if NUM_UARTS >= 2
    printf("  Port 1 (ttyUSB1) @ %d baud, %d ms interval\n", UART2_BAUDRATE, UART2_INTERVAL);
#endif
#if NUM_UARTS >= 3
    printf("  Port 2 (ttyUSB2) @ %d baud, %d ms interval\n", UART3_BAUDRATE, UART3_INTERVAL);
#endif
#if NUM_UARTS >= 4
    printf("  Port 3 (ttyUSB3) @ %d baud, %d ms interval\n", UART4_BAUDRATE, UART4_INTERVAL);
#endif
    printf("\n");
    
    // Configure UARTs based on NUM_UARTS
    uart_config_t configs[NUM_UARTS];
    
#if NUM_UARTS >= 1
    configs[0].port = UART1_PORT;
    configs[0].baudrate = UART1_BAUDRATE;
    configs[0].fd = -1;
    configs[0].data = data1;
    configs[0].size = sizeof(data1);
    configs[0].interval_ms = UART1_INTERVAL;
    configs[0].name = "UART1";
#endif
    
#if NUM_UARTS >= 2
    configs[1].port = UART2_PORT;
    configs[1].baudrate = UART2_BAUDRATE;
    configs[1].fd = -1;
    configs[1].data = data2;
    configs[1].size = sizeof(data2);
    configs[1].interval_ms = UART2_INTERVAL;
    configs[1].name = "UART2";
#endif
    
#if NUM_UARTS >= 3
    configs[2].port = UART3_PORT;
    configs[2].baudrate = UART3_BAUDRATE;
    configs[2].fd = -1;
    configs[2].data = data3;
    configs[2].size = sizeof(data3);
    configs[2].interval_ms = UART3_INTERVAL;
    configs[2].name = "UART3";
#endif
    
#if NUM_UARTS >= 4
    configs[3].port = UART4_PORT;
    configs[3].baudrate = UART4_BAUDRATE;
    configs[3].fd = -1;
    configs[3].data = data4;
    configs[3].size = sizeof(data4);
    configs[3].interval_ms = UART4_INTERVAL;
    configs[3].name = "UART4";
#endif
    
    // Open all UARTs
    for (int i = 0; i < NUM_UARTS; i++) {
        configs[i].fd = open_uart(configs[i].port, configs[i].baudrate);
        if (configs[i].fd < 0) {
            fprintf(stderr, "Failed to open %s\n", configs[i].port);
            // Close previously opened ports
            for (int j = 0; j < i; j++) {
                if (configs[j].fd >= 0) close(configs[j].fd);
            }
            return 1;
        }
    }
    
    printf("All UART ports opened successfully\n");
    printf("Press Ctrl+C to stop...\n\n");
    
    pthread_t threads[NUM_UARTS];
    
    // Create transmit threads
    for (int i = 0; i < NUM_UARTS; i++) {
        if (pthread_create(&threads[i], NULL, uart_transmit_thread, &configs[i]) != 0) {
            fprintf(stderr, "Failed to create thread %d\n", i);
            keep_running = 0;
        }
    }
    
    // Wait for all threads to complete
    for (int i = 0; i < NUM_UARTS; i++) {
        pthread_join(threads[i], NULL);
    }
    
    // Close all ports
    for (int i = 0; i < NUM_UARTS; i++) {
        if (configs[i].fd >= 0) close(configs[i].fd);
    }
    
    printf("\nTransmission stopped\n");
    return 0;
}
