// cmd_stats.c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <stdbool.h>
#include <errno.h>
#include <stdlib.h>
#include <linux/can.h>
#include "../global_options.h"
#include "../can_interface.h"
#include "../log.h"
#include "../utils.h"

#define MAX_CAN_IDS 256

static volatile bool running = true;

typedef struct {
    uint32_t can_id;        // Raw CAN ID (includes flag bits)
    uint32_t count;         // Frames counted within the current window
    uint32_t last_time;     // Last rate update timestamp (ms)
    float rate;             // Calculated frequency (Hz)
    bool is_extended;       // True when using extended identifiers
    bool active;            // Entry currently in use
    uint8_t dlc;
} can_frame_stats_t;

static can_frame_stats_t frame_stats[MAX_CAN_IDS] = {0};
static uint32_t total_count = 0;
static uint32_t last_total_time = 0;
static float total_rate = 0.0f;
static int active_ids = 0;

static void signal_handler(int sig)
{
    (void)sig;
    running = false;
    printf("\nStopping statistics monitor...\n");
}

static int install_signal_handlers(void)
{
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; // do not restart syscalls; we want select/read to exit

    if (sigaction(SIGINT, &sa, NULL) == -1) {
        log_error("Failed to install SIGINT handler: %s", strerror(errno));
        return -1;
    }
    if (sigaction(SIGTERM, &sa, NULL) == -1) {
        log_error("Failed to install SIGTERM handler: %s", strerror(errno));
        return -1;
    }
    return 0;
}

static int compare_stats_by_id(const void *lhs, const void *rhs)
{
    const can_frame_stats_t *const *a = lhs;
    const can_frame_stats_t *const *b = rhs;
    if ((*a)->can_id < (*b)->can_id) return -1;
    if ((*a)->can_id > (*b)->can_id) return 1;
    if ((*a)->is_extended == (*b)->is_extended) return 0;
    return (*a)->is_extended ? 1 : -1;
}



// Timestamp helper provided by utils_get_timestamp_ms()

static int find_or_create_stats_entry(uint32_t can_id, bool is_extended)
{
    for (int i = 0; i < MAX_CAN_IDS; i++) {
        if (frame_stats[i].active && 
            frame_stats[i].can_id == can_id && 
            frame_stats[i].is_extended == is_extended) {
            return i;
        }
    }
    
    for (int i = 0; i < MAX_CAN_IDS; i++) {
        if (!frame_stats[i].active) {
            frame_stats[i].can_id = can_id;
            frame_stats[i].is_extended = is_extended;
            frame_stats[i].active = true;
            frame_stats[i].count = 0;
            frame_stats[i].rate = 0.0f;
            frame_stats[i].last_time = utils_get_timestamp_ms();
            active_ids++;
            return i;
        }
    }
    
    return -1; // Table is full
}

static void update_frame_stats(const struct can_frame *frame)
{
    uint32_t now = utils_get_timestamp_ms();
    bool is_extended = (frame->can_id & CAN_EFF_FLAG) ? true : false;
    uint32_t clean_id = is_extended ? (frame->can_id & CAN_EFF_MASK) : (frame->can_id & CAN_SFF_MASK);
    
    total_count++;
    if (now - last_total_time >= 1000) {
        total_rate = total_count * 1000.0f / (now - last_total_time);
        total_count = 0;
        last_total_time = now;
    }
    
    int index = find_or_create_stats_entry(clean_id, is_extended);
    if (index >= 0) {
        can_frame_stats_t *stat = &frame_stats[index];
        stat->count++;
        stat->dlc = frame->len;
        
        if (now - stat->last_time >= 1000) {
            stat->rate = stat->count * 1000.0f / (now - stat->last_time);
            stat->count = 0;
            stat->last_time = now;
        }
    }
}
static void display_frame_statistics(const char *ifname)
{
    printf("\033[2J\033[H");
    printf("=== HiPNUC CAN frame statistics ===\n");
    printf("Interface: %s | Total rate: %.1f Hz | Active IDs: %d\n\n",
           ifname, total_rate, active_ids);
    
    printf("CAN ID     | Type | Rate (Hz) | DLC\n");
    printf("-----------+------+-----------+-----\n");
    
    can_frame_stats_t *active_stats[MAX_CAN_IDS];
    int active_count = 0;
    for (int i = 0; i < MAX_CAN_IDS; i++) {
        if (frame_stats[i].active) {
            active_stats[active_count++] = &frame_stats[i];
        }
    }
    
    qsort(active_stats, active_count, sizeof(active_stats[0]), compare_stats_by_id);
    
    for (int i = 0; i < active_count; i++) {
        const can_frame_stats_t *stat = active_stats[i];
        char id_buf[16];
        if (stat->is_extended) {
            snprintf(id_buf, sizeof(id_buf), "0x%08X", stat->can_id);
        } else {
            snprintf(id_buf, sizeof(id_buf), "0x%03X", stat->can_id);
        }
        printf("%-10s | %-4s | %9.1f | %d\n",
               id_buf,
               stat->is_extended ? "EXT" : "STD",
               stat->rate,
               stat->dlc);
    }
    
    printf("-----------+------+-----------+-----\n");
    printf("Total: %d active CAN IDs\n", active_ids);
    printf("Press Ctrl+C to exit.\n");
}
int cmd_stats(GlobalOptions *opts, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    
    int sockfd = can_open_socket(opts->can_interface);
    if (sockfd < 0) {
        utils_print_can_setup_hint(opts->can_interface);
        return -1;
    }
    
    printf("HiPNUC CAN frame statistics\n");
    printf("Interface: %s\n", opts->can_interface);
    printf("Listening to every frame on the bus...\n");
    printf("Note: stats mode does not filter by node ID.\n\n");
    
    if (install_signal_handlers() < 0) {
        can_close_socket(sockfd);
        return -1;
    }
    
    sleep(1);

    uint32_t last_display_time = utils_get_timestamp_ms();
    last_total_time = last_display_time;

    while (running) {
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(sockfd, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100 ms refresh cadence
        
        int select_result = select(sockfd + 1, &readfds, NULL, NULL, &timeout);
        
        if (select_result > 0 && FD_ISSET(sockfd, &readfds)) {
            struct can_frame frame;
            int result = can_receive_frame(sockfd, &frame);
            
            if (result > 0) {
                update_frame_stats(&frame);
            } else if (result < 0) {
                break;
            }
        } else if (select_result < 0) {
            if (errno == EINTR) {
                if (!running) {
                    break;
                }
                continue;
            } else {
                log_error("select() failed: %s", strerror(errno));
                break;
            }
        }
        uint32_t now = utils_get_timestamp_ms();
        if (now - last_display_time >= 100) {
            display_frame_statistics(opts->can_interface);
            last_display_time = now;
        }
    }
    
    can_close_socket(sockfd);
    return 0;
}
