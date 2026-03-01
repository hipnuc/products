#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../can_interface.h"
#include "../config.h"
#include "../help.h"
#include "../log.h"
#include "../reg_seq.h"
#include "../commands.h"

static const reg_seq_step_t reset_steps[] = {
    { REG_SEQ_WRITE, 0x00, 0xFF, 0 },
};

static const reg_seq_step_t save_steps[] = {
    { REG_SEQ_WRITE, 0x00, 0x00, 1000 },
};

static const reg_seq_step_t version_steps[] = {
    { REG_SEQ_READ, 0x78, 0, 0 },
};

static const reg_seq_cmd_t known_actions[] = {
    { "reset", "Reset device (write 0xFF to 0x00)", reset_steps, sizeof(reset_steps)/sizeof(reset_steps[0]) },
    { "save", "Save configuration (write 0x00 to 0x00)", save_steps, sizeof(save_steps)/sizeof(save_steps[0]) },
    { "version", "Read firmware version register (0x78)", version_steps, sizeof(version_steps)/sizeof(version_steps[0]) },
    { NULL, NULL, NULL, 0 }
};

static bool action_requires_confirmation(const reg_seq_cmd_t *action)
{
    if (!action || !action->name) {
        return true;
    }
    return strcmp(action->name, "version") != 0;
}

static void print_usage(void)
{
    printf("Usage: canhost action run <name> [--yes]\n");
    printf("Available actions:\n");
    for (int i = 0; known_actions[i].name; i++) {
        printf("  %-12s : %s\n", known_actions[i].name, known_actions[i].description);
    }
}

int cmd_action(int argc, char *argv[])
{
    if (argc < 3 || strcmp(argv[1], "run") != 0) {
        help_print_arg_error_json("action", "usage: action run <name> [--yes]");
        print_usage();
        return CANHOST_EXIT_INVALID_ARGS;
    }

    const char *action_name = argv[2];
    bool confirmed = false;
    for (int i = 3; i < argc; ++i) {
        if (strcmp(argv[i], "--yes") == 0) {
            confirmed = true;
        } else {
            help_print_arg_error_json("action", "unknown option, only --yes is supported");
            return CANHOST_EXIT_INVALID_ARGS;
        }
    }

    const reg_seq_cmd_t *selected_action = NULL;
    for (int i = 0; known_actions[i].name; i++) {
        if (strcmp(action_name, known_actions[i].name) == 0) {
            selected_action = &known_actions[i];
            break;
        }
    }

    if (!selected_action) {
        help_print_arg_error_json("action", "unknown action name");
        print_usage();
        return CANHOST_EXIT_INVALID_ARGS;
    }

    if (action_requires_confirmation(selected_action) && !confirmed) {
        help_print_arg_error_json("action", "dangerous action requires --yes confirmation");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    uint8_t target_nodes[32];
    int target_count = config_get_target_nodes(target_nodes, 32);
    if (target_count <= 0) {
        help_print_arg_error_json("action", "no target node configured");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    uint8_t host_sa = config_get_sync_sa();
    const char *ifname = config_get_interface();
    int fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    int overall_result = CANHOST_EXIT_OK;
    for (int i = 0; i < target_count; i++) {
        uint8_t node = target_nodes[i];
        if (target_count > 1) {
            log_debug("--- Action on node %d ---", node);
        }
        if (reg_seq_execute(fd, node, host_sa, selected_action) < 0) {
            log_error("Action failed on node %d", node);
            overall_result = CANHOST_EXIT_RUNTIME_ERROR;
        }
    }

    can_close_socket(fd);
    return overall_result;
}

