// cmd_read.c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <stdbool.h>
#include "../global_options.h"
#include "../can_interface.h"
#include "../log.h"
#include "../utils.h"
#include "hipnuc_can_parser.h"


#define MAX_MSG_TYPES 15  // Max number of concurrent message types

typedef struct {
    int msg_type;
    can_sensor_data_t data;
    uint32_t last_update_time_ms;
    bool has_data;
    int display_line;
} msg_cache_t;

static volatile bool running = true;
static msg_cache_t msg_cache[MAX_MSG_TYPES];
static int active_msg_count = 0;

static void signal_handler(int sig)
{
    (void)sig;
    running = false;
    printf("\nStopping reader...\n");
}

// Timestamp helper is provided by utils_get_timestamp_ms()

static void init_msg_cache(void)
{
    memset(msg_cache, 0, sizeof(msg_cache));
    active_msg_count = 0;
}

static int find_or_create_msg_cache(int msg_type)
{
    for (int i = 0; i < active_msg_count; i++) {
        if (msg_cache[i].msg_type == msg_type) {
            return i;
        }
    }
    
    if (active_msg_count < MAX_MSG_TYPES) {
        int index = active_msg_count++;
        msg_cache[index].msg_type = msg_type;
        msg_cache[index].has_data = false;
        msg_cache[index].display_line = index + 4; // Table header consumes first rows
        return index;
    }
    
    return -1; // Cache exhausted
}

static void update_msg_cache(int msg_type, const can_sensor_data_t *data)
{
    int index = find_or_create_msg_cache(msg_type);
    if (index >= 0) {
        msg_cache[index].data = *data;
        msg_cache[index].last_update_time_ms = utils_get_timestamp_ms();
        msg_cache[index].has_data = true;
    }
}

static void display_sensor_data(const char *ifname, uint8_t node_id, int msg_type, const can_sensor_data_t *data)
{
    (void)msg_type;
    (void)data;
    
    static bool first_display = true;
    
    if (first_display) {
        printf("\033[2J\033[H");
        printf("=== HiPNUC IMU CAN Live View ===\n");
        printf("Interface: %s | Node ID: %d\n", ifname, node_id);
        printf("Message Type  | CAN ID   | Latest data\n");
        printf("--------------+----------+---------------------------------------------------\n");
        first_display = false;
    }
    
    for (int i = 0; i < active_msg_count; i++) {
        msg_cache_t *cache = &msg_cache[i];
        
        if (!cache->has_data) {
            continue;
        }
        
        printf("\033[%d;1H", cache->display_line);
        
        char data_str[200];
        hipnuc_can_format_data(cache->msg_type, &cache->data, data_str, sizeof(data_str));
        
        printf("\033[K");
        
        char can_id_str[12];
        if (cache->data.is_extended) {
            snprintf(can_id_str, sizeof(can_id_str), "0x%08X", cache->data.can_id & HIPNUC_CAN_EFF_MASK);
        } else {
            snprintf(can_id_str, sizeof(can_id_str), "0x%03X", cache->data.can_id & HIPNUC_CAN_SFF_MASK);
        }
        
        printf("%-12s  | %-8s | %s", 
               hipnuc_can_get_msg_type_name(cache->msg_type),
               can_id_str,
               data_str);
    }
    
    int status_line = 4 + MAX_MSG_TYPES + 2;
    printf("\033[%d;1H", status_line);
    printf("\033[K");
    printf("Active message types: %d | Press Ctrl+C to exit...", active_msg_count);
    
    fflush(stdout);
}

int cmd_read(GlobalOptions *opts, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    
    // Interface readiness already validated by commands.c
    
    int sockfd = can_open_socket(opts->can_interface);
    if (sockfd < 0) {
        utils_print_can_setup_hint(opts->can_interface);
        return -1;
    }
    
    printf("HiPNUC sensor data monitor\n");
    printf("Interface: %s\n", opts->can_interface);
    printf("Node ID: %d\n", opts->node_id);
    printf("Listening for CAN frames and decoding sensor payloads...\n");
    printf("Tip: run 'canhost stats' for raw frame rates.\n\n");
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    init_msg_cache();
    
    sleep(2);
    
    uint32_t last_display_time = utils_get_timestamp_ms();
    
    while (running) {
        struct can_frame frame;
        int result = can_receive_frame(sockfd, &frame);
        
        if (result > 0) {
            can_sensor_data_t can_sensor_data;
            hipnuc_can_frame_t hipnuc_frame;
            utils_linux_can_to_hipnuc_can(&frame, &hipnuc_frame);
            int msg_type = hipnuc_can_parse_frame(&hipnuc_frame, &can_sensor_data, opts->node_id);

            if (msg_type != CAN_MSG_UNKNOWN && msg_type != CAN_MSG_ERROR) {
                update_msg_cache(msg_type, &can_sensor_data);
            }
        } else if (result < 0) {
            break;
        }
        
        uint32_t now = utils_get_timestamp_ms();
        if (now - last_display_time >= 100) {
            display_sensor_data(opts->can_interface, opts->node_id, 0, NULL);
            last_display_time = now;
        }
    }
    
    can_close_socket(sockfd);
    return 0;
}
