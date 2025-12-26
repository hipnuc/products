#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <linux/can.h>
#include <errno.h>
#include "../can_interface.h"
#include "../log.h"
#include "../utils.h"
#include "../help.h"
#include "../config.h"
#include "hipnuc_can_common.h"

static volatile sig_atomic_t g_run = 1;

static void on_signal(int sig)
{
    (void)sig;
    g_run = 0;
}


int cmd_sync(int argc, char *argv[])
{
    unsigned long count_target = 0;
    uint32_t filter_pgn = 0;

    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--count") == 0) && i + 1 < argc) {
            count_target = strtoul(argv[++i], NULL, 0);
        } else if ((strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--pgn") == 0) && i + 1 < argc) {
            filter_pgn = (uint32_t)strtoul(argv[++i], NULL, 0);
        }
    }

    config_sync_item_t items_all[32];
    int item_cnt_all = config_get_sync_items(items_all, 32);
    uint8_t target_nodes[32];
    int target_count = config_get_target_nodes(target_nodes, 32);
    uint8_t host_sa = config_get_sync_sa();
    config_sync_item_t items[32];
    int item_cnt = 0;
    if (filter_pgn != 0) {
        for (int i = 0; i < item_cnt_all; ++i) {
            if (items_all[i].pgn == filter_pgn) {
                items[item_cnt++] = items_all[i];
            }
        }
    } else {
        for (int i = 0; i < item_cnt_all; ++i) items[i] = items_all[i];
        item_cnt = item_cnt_all;
    }
    if (item_cnt <= 0) {
        log_error("No sync items configured in canhost.ini (use sync.<pgn>=<period_ms>)");
        return -1;
    }

    const char *ifname = config_get_interface();
    int fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        return -1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    printf("Sync config: if=%s nodes=%d sa=%u items=%d\n", ifname, target_count, (unsigned)host_sa, item_cnt);
    for (int i = 0; i < item_cnt; ++i) {
        printf("  PGN=0x%X period=%ums\n", (unsigned)items[i].pgn, (unsigned)items[i].period_ms);
    }

    uint32_t now_ms = utils_get_timestamp_ms();
    uint32_t next_due[32];
    uint64_t sent_total = 0;
    uint64_t last_print_total = 0;
    uint32_t last_print_ms = now_ms;
    uint32_t sent_counts[32] = {0};
    for (int i = 0; i < item_cnt; ++i) {
        next_due[i] = (count_target >= 1) ? now_ms : (now_ms + items[i].period_ms);
    }

    while (g_run) {
        now_ms = utils_get_timestamp_ms();
        uint32_t min_sleep = 50;
        for (int i = 0; i < item_cnt; ++i) {
            if ((int32_t)(now_ms - next_due[i]) >= 0) {
                for (int k = 0; k < target_count; ++k) {
                    hipnuc_can_frame_t cfg;
                    hipnuc_j1939_build_sync(target_nodes[k], host_sa, items[i].pgn, &cfg);
                    struct can_frame f;
                    utils_hipnuc_can_to_linux_can(&cfg, &f);
                    ssize_t w = write(fd, &f, sizeof(f));
                    if (w != sizeof(f)) { log_error("send failed"); g_run = 0; break; }
                    sent_total++;
                }
                if (!g_run) break;
                
                next_due[i] += items[i].period_ms;
                if (count_target >= 1) {
                    if (sent_counts[i] < UINT32_MAX) sent_counts[i]++;
                }
            }
            uint32_t remain = (next_due[i] > now_ms) ? (next_due[i] - now_ms) : 0;
            if (remain < min_sleep) min_sleep = remain;
        }

        if (count_target >= 1) {
            bool done = true;
            for (int i = 0; i < item_cnt; ++i) {
                if (sent_counts[i] < count_target) { done = false; break; }
            }
            if (done) { g_run = 0; }
        }

        if (now_ms - last_print_ms >= 1000) {
            uint64_t delta = sent_total - last_print_total;
            double fps = (double)delta * 1000.0 / (double)(now_ms - last_print_ms);
            printf("\rsent_total=%llu fps=%.1f   ", (unsigned long long)sent_total, fps);
            fflush(stdout);
            last_print_total = sent_total;
            last_print_ms = now_ms;
        }

        if (min_sleep > 0) usleep((useconds_t)min_sleep * 1000);
    }

    printf("\n");
    log_info("Sync trigger done: sent_total=%llu", (unsigned long long)sent_total);
    can_close_socket(fd);
    return 0;
}
