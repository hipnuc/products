// main.c
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "hipnuc_can_parser.h"

typedef struct {
    uint32_t count;
    uint32_t last_time;
    float rate;
    can_sensor_data_t data;
    bool has_data;
} can_msg_stats_t;

static volatile bool running = true;
static can_msg_stats_t msg_stats[100] = {0};  // Large enough for any message type
static uint32_t total_count = 0;
static uint32_t last_total_time = 0;
static float total_rate = 0.0f;

void signal_handler(int sig)
{
    running = false;
    printf("\nShutting down...\n");
}

static uint32_t get_timestamp_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void linux_can_to_hipnuc_can(const struct can_frame *linux_frame, hipnuc_can_frame_t *hipnuc_frame)
{
    hipnuc_frame->can_id = linux_frame->can_id;
    hipnuc_frame->can_dlc = linux_frame->can_dlc;
    memcpy(hipnuc_frame->data, linux_frame->data, 8);
}


// Check CAN interface and show available interfaces
static int check_and_list_can_interfaces(const char *ifname)
{
    DIR *dir;
    struct dirent *entry;
    bool found_can = false;
    bool target_found = false;
    bool target_up = false;
    
    printf("\n");
    printf("=====================================\n");
    printf("     Available CAN Interfaces       \n");
    printf("=====================================\n");
    
    dir = opendir("/sys/class/net");
    if (dir == NULL) {
        printf("  Error: Cannot access network interfaces\n");
        printf("=====================================\n\n");
        return -1;
    }
    
    while ((entry = readdir(dir)) != NULL) {
        // Only real CAN interfaces
        if (strncmp(entry->d_name, "can", 3) == 0 && 
            strncmp(entry->d_name, "vcan", 4) != 0) {
            
            char path[256];
            snprintf(path, sizeof(path), "/sys/class/net/%s/operstate", entry->d_name);
            
            FILE *f = fopen(path, "r");
            if (f) {
                char state[16] = {0};
                fgets(state, sizeof(state), f);
                fclose(f);
                
                char *newline = strchr(state, '\n');
                if (newline) *newline = '\0';
                
                // Check device type
                char type_path[256];
                snprintf(type_path, sizeof(type_path), "/sys/class/net/%s/type", entry->d_name);
                FILE *type_f = fopen(type_path, "r");
                if (type_f) {
                    char type_str[16] = {0};
                    fgets(type_str, sizeof(type_str), type_f);
                    fclose(type_f);
                    
                    if (atoi(type_str) == 280) { // ARPHRD_CAN
                        // Mark with * if this is the target interface
                        char marker = ' ';
                        if (strcmp(entry->d_name, ifname) == 0) {
                            marker = '*';
                            target_found = true;
                            if (strncmp(state, "up", 2) == 0 || strncmp(state, "unknown", 7) == 0) {
                                target_up = true;
                            }
                        }
                        
                        printf(" %c%-10s : %s\n", marker, entry->d_name, state);
                        found_can = true;
                    }
                }
            }
        }
    }
    
    closedir(dir);
    
    printf("=====================================\n");
    printf("Selected: %s\n", ifname);
    
    if (!found_can) {
        printf("\nNo real CAN interfaces found!\n");
        printf("Check if CAN hardware is connected.\n\n");
        printf("CAN Setup Commands:\n");
        printf("  sudo ip link set down can0\n");
        printf("  sudo ip link set can0 type can bitrate 500000\n");
        printf("  sudo ip link set up can0\n");
        printf("  ip link show can0\n\n");
        return -1;
    }
    
    if (!target_found) {
        printf("\nError: Interface '%s' not found!\n", ifname);
        printf("Please choose one from the list above.\n\n");
        return -1;
    }
    
    if (!target_up) {
        printf("\nWarning: Interface '%s' is not up!\n", ifname);
        printf("Setup commands:\n");
        printf("  sudo ip link set down %s\n", ifname);
        printf("  sudo ip link set %s type can bitrate 500000\n", ifname);
        printf("  sudo ip link set up %s\n", ifname);
        printf("  ip link show %s\n\n", ifname);
        return -1;
    }
    
    printf("Interface '%s' is ready!\n\n", ifname);
    return 0;
}

// Update statistics for a message type
static void update_stats(int msg_type, const can_sensor_data_t *data)
{
    uint32_t now = get_timestamp_ms();
    
    // Update total stats
    total_count++;
    if (now - last_total_time >= 1000) {
        total_rate = total_count * 1000.0f / (now - last_total_time);
        total_count = 0;
        last_total_time = now;
    }
    
    // Update per-message stats
    if (msg_type > 0 && msg_type < 100) {
        can_msg_stats_t *stat = &msg_stats[msg_type];
        stat->count++;
        stat->has_data = true;
        
        if (data) {
            stat->data = *data;
        }
        
        // Calculate rate every second
        if (now - stat->last_time >= 1000) {
            stat->rate = stat->count * 1000.0f / (now - stat->last_time);
            stat->count = 0;
            stat->last_time = now;
        }
    }
}

static void display_statistics(const char *ifname, uint8_t node_id)
{
    printf("\033[2J\033[H"); // Clear screen
    printf("=== HiPNUC IMU CAN Parser ===\n");
    printf("Interface: %s | Node ID: %d | Total: %.1f Hz\n\n", 
           ifname, node_id, total_rate);
    
    printf("Message Type  | Rate (Hz) | Frame ID & Data\n");
    printf("--------------+-----------+--------------------------------------------------------\n");
    
    // Show all message types that have data
    for (int msg_type = 1; msg_type < 100; msg_type++) {
        can_msg_stats_t *stat = &msg_stats[msg_type];
        
        if (stat->has_data) {
            char data_str[200];
            hipnuc_can_format_data(msg_type, &stat->data, data_str, sizeof(data_str));
            
            printf("%-12s  | %7.1f   | %s\n", 
                   hipnuc_can_get_msg_type_name(msg_type), stat->rate, data_str);
        }
    }
    
    printf("\nPress Ctrl+C to exit...\n");
}


// Print usage information
static void print_usage(const char *prog_name)
{
    printf("Usage: %s [interface] [node_id]\n", prog_name);
    printf("Options:\n");
    printf("  interface   CAN interface name (default: can0)\n");
    printf("  node_id     CANopen node ID (default: 8)\n");
    printf("  -h          Show this help\n");
    printf("\nExamples:\n");
    printf("  %s can0 8           # Listen on can0, node ID 8\n", prog_name);
}

int main(int argc, char *argv[])
{
    uint32_t last_display_time = 0;
    const char *ifname = "can0";
    uint8_t node_id = 8;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (argv[i][0] != '-') {
            if (i == 1) {
                ifname = argv[i];
            } else if (i == 2) {
                node_id = atoi(argv[i]);
            }
        }
    }
    
    printf("HiPNUC sensor data CAN Parser\n");
    
    // Check CAN interface and show available interfaces
    if (check_and_list_can_interfaces(ifname) != 0) {
        return 1;
    }
    
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Create CAN socket
    int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0) {
        perror("socket");
        return 1;
    }
    
    // Bind to CAN interface
    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        close(sockfd);
        return 1;
    }
    
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sockfd);
        return 1;
    }
    
    printf("Successfully connected to %s\n", ifname);
    printf("CANopen Node ID: %d\n", node_id);
    printf("Listening for CAN frames...\n\n");

    // Give user a moment to see the connection status
    sleep(1);
    
    // Initialize timing
    last_display_time = get_timestamp_ms();
    
    // Main receive loop
    while (running) {
        struct can_frame frame;
        ssize_t nbytes = read(sockfd, &frame, sizeof(frame));
        
        if (nbytes < 0) {
            if (errno == EINTR) continue; // Interrupted by signal
            perror("read");
            break;
        }
        
        if (nbytes < sizeof(frame)) {
            fprintf(stderr, "Incomplete CAN frame\n");
            continue;
        }
        
        // Parse frame with target node ID
        can_sensor_data_t can_sensor_data;
        hipnuc_can_frame_t hipnuc_frame;
        linux_can_to_hipnuc_can(&frame, &hipnuc_frame);
        int msg_type = hipnuc_can_parse_frame(&hipnuc_frame, &can_sensor_data, node_id);

        // Process valid frames from target node
        if (msg_type != CAN_MSG_UNKNOWN && msg_type != CAN_MSG_ERROR) {
            update_stats(msg_type, &can_sensor_data);
        }
                
        // Update display every 50ms
        uint32_t now = get_timestamp_ms();
        if (now - last_display_time >= 50) {
            display_statistics(ifname, node_id);
            last_display_time = now;
        }
    }
    
    close(sockfd);
    return 0;
}
