/* trigger sync [--count N] [--pgn PGN]: send J1939 trigger frames for the
 * PGNs listed in canhost.ini (sync.<pgn>=<period_ms>) to every target node. */
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_interface.h"
#include "commands.h"
#include "config.h"
#include "help.h"
#include "log.h"
#include "utils.h"
#include "hipnuc_j1939.h"

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
        if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--count") == 0) {
            if (i + 1 >= argc) {
                help_print_arg_error_json("trigger sync", "missing value for --count");
                return CANHOST_EXIT_INVALID_ARGS;
            }
            count_target = strtoul(argv[++i], NULL, 0);
        } else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--pgn") == 0) {
            if (i + 1 >= argc) {
                help_print_arg_error_json("trigger sync", "missing value for --pgn");
                return CANHOST_EXIT_INVALID_ARGS;
            }
            filter_pgn = (uint32_t)strtoul(argv[++i], NULL, 0);
        } else {
            help_print_arg_error_json("trigger sync", "unknown option");
            return CANHOST_EXIT_INVALID_ARGS;
        }
    }

    config_sync_item_t items_all[CONFIG_MAX_SYNC_ITEMS];
    int item_cnt_all = config_get_sync_items(items_all, CONFIG_MAX_SYNC_ITEMS);
    uint8_t target_nodes[CONFIG_MAX_NODES];
    int target_count = config_get_target_nodes(target_nodes, CONFIG_MAX_NODES);
    if (target_count <= 0) {
        help_print_arg_error_json("trigger sync", "no target node configured");
        return CANHOST_EXIT_INVALID_ARGS;
    }
    uint8_t host_sa = config_get_sync_sa();

    config_sync_item_t items[CONFIG_MAX_SYNC_ITEMS];
    int item_cnt = 0;
    for (int i = 0; i < item_cnt_all; ++i) {
        if (filter_pgn == 0 || items_all[i].pgn == filter_pgn) {
            items[item_cnt++] = items_all[i];
        }
    }
    if (item_cnt <= 0) {
        help_print_arg_error_json("trigger sync", "no matching sync.<pgn> entry in canhost.ini");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    const char *ifname = config_get_interface();
    int fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    printf("Sync: if=%s nodes=%d sa=0x%02X items=%d\n", ifname, target_count, (unsigned)host_sa, item_cnt);
    for (int i = 0; i < item_cnt; ++i) {
        printf("  PGN=0x%X period=%u ms\n", (unsigned)items[i].pgn, (unsigned)items[i].period_ms);
    }

    uint64_t now = utils_now_ms();
    uint64_t next_due[CONFIG_MAX_SYNC_ITEMS];
    unsigned long sent_counts[CONFIG_MAX_SYNC_ITEMS] = {0};
    uint64_t sent_total = 0, last_print_total = 0;
    uint64_t last_print_ms = now;
    int had_runtime_error = 0;
    for (int i = 0; i < item_cnt; ++i) {
        next_due[i] = count_target >= 1 ? now : now + items[i].period_ms;
    }

    while (g_run) {
        now = utils_now_ms();
        uint64_t min_sleep = 50;
        for (int i = 0; i < item_cnt && g_run; ++i) {
            if (now >= next_due[i]) {
                for (int k = 0; k < target_count; ++k) {
                    hipnuc_can_frame_t req;
                    hipnuc_j1939_build_trigger(target_nodes[k], host_sa, items[i].pgn, &req);
                    if (can_send_frame(fd, &req) < 0) {
                        had_runtime_error = 1;
                        g_run = 0;
                        break;
                    }
                    sent_total++;
                }
                next_due[i] += items[i].period_ms;
                sent_counts[i]++;
            }
            uint64_t remain = next_due[i] > now ? next_due[i] - now : 0;
            if (remain < min_sleep) {
                min_sleep = remain;
            }
        }

        if (count_target >= 1) {
            bool done = true;
            for (int i = 0; i < item_cnt; ++i) {
                if (sent_counts[i] < count_target) {
                    done = false;
                    break;
                }
            }
            if (done) {
                break;
            }
        }

        if (now - last_print_ms >= 1000) {
            double fps = (double)(sent_total - last_print_total) * 1000.0 / (double)(now - last_print_ms);
            printf("\rsent=%llu fps=%.1f   ", (unsigned long long)sent_total, fps);
            fflush(stdout);
            last_print_total = sent_total;
            last_print_ms = now;
        }

        if (min_sleep > 0) {
            utils_delay_ms((uint32_t)min_sleep);
        }
    }

    printf("\n");
    log_info("Sync done: sent=%llu", (unsigned long long)sent_total);
    can_close_socket(fd);
    return had_runtime_error ? CANHOST_EXIT_RUNTIME_ERROR : CANHOST_EXIT_OK;
}
