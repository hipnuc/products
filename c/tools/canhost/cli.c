#include "cli.h"
#include <errno.h>
#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int unsigned_value(const char *text, uint64_t maximum, uint64_t *result)
{
    char *end;
    unsigned long long number;
    if (!text || !isdigit((unsigned char)*text)) return -1;
    errno = 0;
    number = strtoull(text, &end, strncmp(text, "0x", 2) == 0 || strncmp(text, "0X", 2) == 0 ? 16 : 10);
    if (errno || end == text || *end || number > maximum) return -1;
    *result = number;
    return 0;
}

static int milliseconds(const char *text, uint64_t maximum, uint64_t *result)
{
    char *end;
    double number;
    if (!text || !*text || *text == '-' || *text == '+' || isspace((unsigned char)*text)) return -1;
    errno = 0;
    number = strtod(text, &end);
    if (errno || end == text || *end || !isfinite(number) ||
        number < 0.001 || number * 1000.0 >= (double)maximum) return -1;
    *result = (uint64_t)(number * 1000.0 + 0.5);
    return 0;
}

void canhost_help(void)
{
    puts("HiPNUC SocketCAN tools\n"
         "  canhost list\n"
         "  canhost scan -i INTERFACE [--duration SECONDS]\n"
         "  canhost read -i INTERFACE [-n NODE] [--duration SECONDS] [--count N]\n"
         "               [--record FILE] [--overwrite]\n"
         "  canhost reg read ADDRESS -i INTERFACE -n NODE [--timeout 1]\n"
         "  canhost reg write ADDRESS VALUE -i INTERFACE -n NODE [--timeout 1]\n"
         "  canhost sync PGN -i INTERFACE -n NODE [--interval 0.01] [--count N]\n"
         "  canhost update FILE -i INTERFACE -n NODE [--bin]\n\n"
         "Numbers accept decimal or 0x-prefixed hexadecimal. Time values are seconds.\n"
         "Scan listens for measurement traffic for 2 seconds; it sends no request.\n"
         "Read emits one complete JSON sample per line, or writes --record instead.\n"
         "Read without -n accepts all addresses (0..255). Reg/sync use unicast 0..253;\n"
         "update requires one node in 1..127. J1939 requests use host address 85.\n"
         "Configure interface bitrate separately.\n"
         "Duration/count finish the current receive batch; Ctrl-C stops with code 130.");
}

int canhost_parse(int argc, char **argv, canhost_options_t *o)
{
    const char *positional[2];
    int position_count = 0, start = 2, seen = 0;
    enum { IFACE=1, NODE=2, DURATION=8, COUNT=16, RECORD=32, OVERWRITE=64,
           INTERVAL=128, TIMEOUT=256, BINARY=512 };
    int allowed;
    memset(o, 0, sizeof(*o));
    o->node = -1;
    o->source = 85;
    o->interval_ms = 10;
    o->timeout_ms = 1000;
    if (argc < 2 || strcmp(argv[1], "help") == 0 || strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)
        return 1;
    if (strcmp(argv[1], "list") == 0) { o->command = CMD_LIST; allowed = 0; }
    else if (strcmp(argv[1], "scan") == 0) { o->command = CMD_SCAN; allowed = IFACE|DURATION; o->duration_ms = 2000; }
    else if (strcmp(argv[1], "read") == 0) { o->command = CMD_READ; allowed = IFACE|NODE|DURATION|COUNT|RECORD|OVERWRITE; }
    else if (strcmp(argv[1], "sync") == 0) { o->command = CMD_SYNC; allowed = IFACE|NODE|INTERVAL|COUNT; }
    else if (strcmp(argv[1], "update") == 0) { o->command = CMD_UPDATE; allowed = IFACE|NODE|BINARY; }
    else if (strcmp(argv[1], "reg") == 0) {
        if (argc == 2 || strcmp(argv[2], "--help") == 0 || strcmp(argv[2], "-h") == 0) return 1;
        if (strcmp(argv[2], "read") == 0) o->command = CMD_REG_READ;
        else if (strcmp(argv[2], "write") == 0) o->command = CMD_REG_WRITE;
        else goto invalid;
        allowed = IFACE|NODE|TIMEOUT;
        start = 3;
    } else goto invalid;

    for (int i = start; i < argc; ++i) {
        const char *name = argv[i], *value = NULL;
        int flag = 0;
        uint64_t number = 0;
        if (strcmp(name, "--help") == 0 || strcmp(name, "-h") == 0) return 1;
        if (name[0] != '-') {
            if (position_count == 2) goto invalid;
            positional[position_count++] = name;
            continue;
        }
        if (strcmp(name, "-i") == 0 || strcmp(name, "--interface") == 0) flag = IFACE;
        else if (strcmp(name, "-n") == 0 || strcmp(name, "--node") == 0) flag = NODE;
        else if (strcmp(name, "--duration") == 0) flag = DURATION;
        else if (strcmp(name, "--count") == 0) flag = COUNT;
        else if (strcmp(name, "--record") == 0) flag = RECORD;
        else if (strcmp(name, "--overwrite") == 0) flag = OVERWRITE;
        else if (strcmp(name, "--interval") == 0) flag = INTERVAL;
        else if (strcmp(name, "--timeout") == 0) flag = TIMEOUT;
        else if (strcmp(name, "--bin") == 0) flag = BINARY;
        if (!flag || !(allowed & flag) || (seen & flag)) goto invalid;
        seen |= flag;
        if (flag != OVERWRITE && flag != BINARY) {
            if (++i == argc || !*argv[i]) goto invalid;
            value = argv[i];
        }
        switch (flag) {
        case IFACE: if (strlen(value) >= 16 || value[0] == '-') goto invalid; o->interface = value; break;
        case NODE: if (unsigned_value(value, 255, &number)) goto invalid; o->node = (int)number; break;
        case COUNT: if (unsigned_value(value, UINT64_MAX, &number) || !number) goto invalid; o->count = number; break;
        case DURATION: if (milliseconds(value, UINT64_MAX, &number)) goto invalid; o->duration_ms = number; break;
        case INTERVAL: if (milliseconds(value, UINT32_MAX, &number)) goto invalid; o->interval_ms = (uint32_t)number; break;
        case TIMEOUT: if (milliseconds(value, INT_MAX, &number)) goto invalid; o->timeout_ms = (uint32_t)number; break;
        case RECORD: if (value[0] == '-') goto invalid; o->record_path = value; break;
        case OVERWRITE: o->overwrite = 1; break;
        case BINARY: o->raw_binary = 1; break;
        }
    }
    if (o->command != CMD_LIST && !o->interface) goto invalid;
    if (o->overwrite && !o->record_path) goto invalid;
    if (o->command == CMD_REG_READ || o->command == CMD_REG_WRITE || o->command == CMD_SYNC) {
        if (o->node < 0 || o->node > 253 || o->node == o->source) goto invalid;
    }
    if (o->command == CMD_UPDATE && (o->node < 1 || o->node > 127)) goto invalid;
    if (o->command == CMD_UPDATE) {
        if (position_count != 1) goto invalid;
        o->image_path = positional[0];
    } else if (o->command == CMD_SYNC) {
        uint64_t number;
        if (position_count != 1 || unsigned_value(positional[0], 0x3ffff, &number)) goto invalid;
        o->pgn = (uint32_t)number;
        if ((o->pgn & 0xff00) < 0xf000 && (o->pgn & 0xff)) goto invalid;
    } else if (o->command == CMD_REG_READ || o->command == CMD_REG_WRITE) {
        uint64_t number;
        if (position_count != (o->command == CMD_REG_READ ? 1 : 2) ||
            unsigned_value(positional[0], UINT16_MAX, &number)) goto invalid;
        o->address = (uint16_t)number;
        if (o->command == CMD_REG_WRITE) {
            if (unsigned_value(positional[1], UINT32_MAX, &number)) goto invalid;
            o->value = (uint32_t)number;
        }
    } else if (position_count) goto invalid;
    return 0;
invalid:
    fputs("Error: invalid or missing argument. Run canhost --help.\n", stderr);
    return -1;
}
