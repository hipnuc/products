#ifndef CANHOST_CLI_H
#define CANHOST_CLI_H
#include <signal.h>
#include <stdint.h>

typedef enum { CMD_LIST, CMD_SCAN, CMD_READ, CMD_REG_READ, CMD_REG_WRITE, CMD_SYNC, CMD_UPDATE } canhost_command_t;
typedef struct {
    canhost_command_t command;
    const char *interface;
    const char *record_path;
    const char *image_path;
    int node;                         /* -1: receive from all source addresses */
    uint8_t source;                    /* 0x55: product firmware replies to this host address */
    uint16_t address;
    uint32_t value, pgn;
    uint64_t count, duration_ms;
    uint32_t interval_ms, timeout_ms;
    int overwrite, raw_binary;
} canhost_options_t;

/* Parse only: no files, sockets, or signals. 0 options, 1 help, -1 error. */
int canhost_parse(int argc, char **argv, canhost_options_t *options);
void canhost_help(void);
int canhost_run(const canhost_options_t *options);
int canhost_update(const canhost_options_t *options);
extern volatile sig_atomic_t canhost_stop;
#endif
