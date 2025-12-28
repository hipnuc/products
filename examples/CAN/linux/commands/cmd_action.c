#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "../can_interface.h"
#include "../log.h"
#include "../help.h"
#include "../utils.h"
#include "../config.h"
#include "../reg_seq.h"

// --- Sequence Definitions ---

static const reg_seq_step_t reset_steps[] = {
    { REG_SEQ_WRITE, 0x00, 0xFF, 0 },
};

static const reg_seq_step_t save_steps[] = {
    { REG_SEQ_WRITE, 0x00, 0x00, 1000 },
};

static const reg_seq_step_t version_steps[] = {
    { REG_SEQ_READ, 0x78, 0, 0 },
};

// Example: Magnetic Calibration (as described by user: write reg, then poll status)
// This is a placeholder/example sequence.
// Assume: Write 0x01 to 0x10 to start. Poll 0x11 bit 0 until it becomes 1.
static const reg_seq_step_t mag_calib_steps[] = {
    { REG_SEQ_WRITE, 0x08, 0x01, 0 },           // Start Calibration
    { REG_SEQ_DELAY, 0, 0, 100 },               // Wait 100ms
    { REG_SEQ_POLL,  0x0B, 100, 30000 },        // Poll 0x0B until it is 100. Timeout 30s.
};

static const reg_seq_cmd_t known_actions[] = {
    { "reset", "Reset the device (Write 0xFF to 0x00)", reset_steps, sizeof(reset_steps)/sizeof(reset_steps[0]) },
    { "save", "Save configuration (Write 0x00 to 0x00)", save_steps, sizeof(save_steps)/sizeof(save_steps[0]) },
    { "version", "Read Firmware Version (Read 0x78)", version_steps, sizeof(version_steps)/sizeof(version_steps[0]) },
    { "mag_calib", "Magnetic Calibration (Example)", mag_calib_steps, sizeof(mag_calib_steps)/sizeof(mag_calib_steps[0]) },
    { NULL, NULL, NULL, 0 }
};

static void print_usage(void)
{
    printf("Usage: canhost action <name>\n");
    printf("Available actions:\n");
    for (int i = 0; known_actions[i].name; i++) {
        printf("  %-12s : %s\n", known_actions[i].name, known_actions[i].description);
    }
}

int cmd_action(int argc, char *argv[])
{
    if (argc < 2) {
        print_usage();
        return -1;
    }

    const char *action_name = argv[1];
    const reg_seq_cmd_t *selected_action = NULL;

    for (int i = 0; known_actions[i].name; i++) {
        if (strcmp(action_name, known_actions[i].name) == 0) {
            selected_action = &known_actions[i];
            break;
        }
    }

    if (!selected_action) {
        log_error("Unknown action: %s", action_name);
        print_usage();
        return -1;
    }

    // Get configuration
    uint8_t target_nodes[32];
    int target_count = config_get_target_nodes(target_nodes, 32);
    uint8_t host_sa = config_get_sync_sa();
    const char *ifname = config_get_interface();

    // Open socket
    int fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        return -1;
    }

    int overall_result = 0;

    for (int i = 0; i < target_count; i++) {
        uint8_t node = target_nodes[i];
        if (target_count > 1) {
            log_debug("--- Action on Node %d ---", node);
        }
        
        if (reg_seq_execute(fd, node, host_sa, selected_action) < 0) {
            log_error("Action failed on node %d", node);
            overall_result = -1;
        }
    }

    can_close_socket(fd);
    return overall_result;
}
