#include "cli.h"
#include "can_interface.h"
#include "hipnuc_j1939.h"
#include "hipnuc_json.h"
#include "j1939_reg_api.h"
#include "utils.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int canhost_write_sample(FILE *file, const hipnuc_sample_t *sample, uint64_t timestamp,
                         char **buffer, size_t *capacity)
{
    int required = hipnuc_json_sample(sample, NULL, 0);
    if (required < 0) { fputs("Error: cannot serialize sample.\n", stderr); return -1; }
    if ((size_t)required + 1 > *capacity) {
        char *next = realloc(*buffer, (size_t)required + 1);
        if (!next) { fputs("Error: cannot allocate JSON buffer.\n", stderr); return -1; }
        *buffer = next;
        *capacity = (size_t)required + 1;
    }
    if (hipnuc_json_sample(sample, *buffer, *capacity) != required) return -1;
    // The SDK returns a complete JSON object. Insert host receive time once.
    if (fprintf(file, "{\"rx_time_us\":%llu,%s\n", (unsigned long long)timestamp, *buffer + 1) < 0) {
        fprintf(stderr, "Error: write failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static int open_interface(const canhost_options_t *o)
{
    int fd = can_open_socket(o->interface);
    if (fd < 0) fprintf(stderr, "Error: cannot open %s: %s. Check interface name and link configuration.\n",
                        o->interface, strerror(errno));
    else fprintf(stderr, "Listening on %s.\n", o->interface);
    return fd;
}

static int list_interfaces(void)
{
    can_interface_info_t items[256];
    int count = can_list_interfaces(items, 256);
    if (count < 0) { fputs("Error: cannot enumerate CAN interfaces.\n", stderr); return 1; }
    if (!count) { puts("No CAN interfaces found."); return 0; }
    puts("INTERFACE        STATE");
    for (int i = 0; i < count; ++i) printf("%-16s %s\n", items[i].name, items[i].state);
    return ferror(stdout) ? 1 : 0;
}

static int read_samples(const canhost_options_t *o)
{
    FILE *output = stdout;
    char *json = NULL;
    size_t capacity = 0;
    uint64_t samples = 0, received = 0, invalid = 0;
    int result = 0;
    if (o->record_path) {
        output = fopen(o->record_path, o->overwrite ? "w" : "wx");
        if (!output) {
            fprintf(stderr, "Error: cannot create %s: %s\n", o->record_path, strerror(errno));
            return 1;
        }
    }
    int fd = open_interface(o);
    if (fd < 0) { result = 1; goto done; }
    if (o->record_path) fprintf(stderr, "Recording JSONL to %s. Ctrl-C stops after the current batch.\n", o->record_path);
    uint64_t start = utils_now_ms(), last_flush = start;
    while (!canhost_stop) {
        can_rx_frame_t frames[256];
        int timeout = 100;
        uint64_t elapsed = utils_now_ms() - start;
        if (o->duration_ms) {
            if (elapsed >= o->duration_ms) break;
            if (o->duration_ms - elapsed < (uint64_t)timeout) timeout = (int)(o->duration_ms - elapsed);
        }
        int count = can_receive_frames(fd, frames, 256, timeout);
        if (count < 0) { fputs("Error: CAN receive failed.\n", stderr); result = 1; break; }
        received += (uint64_t)count;
        // Finish frames already received, even after Ctrl-C or a count limit.
        for (int i = 0; i < count; ++i) {
            hipnuc_sample_t sample;
            int type = hipnuc_j1939_parse(&frames[i].frame, &sample, NULL);
            if (type < 0) { ++invalid; continue; }
            if (!type || (o->node >= 0 && sample.node_id != o->node)) continue;
            if (canhost_write_sample(output, &sample, frames[i].timestamp_us, &json, &capacity)) {
                result = 1;
                break;
            }
            ++samples;
        }
        if (result) break;
        uint64_t now = utils_now_ms();
        if (now - last_flush >= 1000) {
            if (fflush(output) != 0) {
                fprintf(stderr, "Error: flush failed: %s\n", strerror(errno));
                result = 1;
                break;
            }
            last_flush = now;
        }
        if (o->count && samples >= o->count) break;
    }
    if (!samples && !canhost_stop && !result) {
        fputs("Error: no valid samples received; check source address, bitrate and device output.\n", stderr);
        result = 1;
    }
    can_close_socket(fd);
done:
    free(json);
    if (fflush(output) != 0) {
        fprintf(stderr, "Error: flush failed: %s\n", strerror(errno));
        result = 1;
    }
    if (output != stdout && fclose(output) != 0) {
        fprintf(stderr, "Error: close failed: %s\n", strerror(errno));
        result = 1;
    }
    fprintf(stderr, "Received %llu frames; wrote %llu samples; invalid %llu.\n",
            (unsigned long long)received, (unsigned long long)samples, (unsigned long long)invalid);
    return result ? result : canhost_stop ? 130 : 0;
}

static int scan(const canhost_options_t *o)
{
    unsigned char found[256] = {0};
    int fd = open_interface(o), result = 0;
    unsigned int count = 0;
    if (fd < 0) return 1;
    fprintf(stderr, "Passive scan for %g seconds; no requests are sent.\n", o->duration_ms / 1000.0);
    uint64_t start = utils_now_ms();
    while (!canhost_stop) {
        can_rx_frame_t frames[64];
        uint64_t elapsed = utils_now_ms() - start;
        if (elapsed >= o->duration_ms) break;
        uint64_t remaining = o->duration_ms - elapsed;
        int received = can_receive_frames(fd, frames, 64, remaining < 100 ? (int)remaining : 100);
        if (received < 0) { fputs("Error: CAN receive failed.\n", stderr); result = 1; break; }
        for (int i = 0; i < received; ++i) {
            hipnuc_sample_t sample;
            if (hipnuc_j1939_parse(&frames[i].frame, &sample, NULL) <= 0 || found[sample.node_id]) continue;
            found[sample.node_id] = 1;
            ++count;
            printf("node=%u protocol=%s\n", (unsigned)sample.node_id,
                   sample.source == HIPNUC_SOURCE_CANFD83 ? "CANFD83" : "J1939");
        }
        if (ferror(stdout)) { result = 1; break; }
    }
    if (!count && !canhost_stop && !result) {
        fputs("No recognized measurement traffic found.\n", stderr);
        result = 1;
    }
    can_close_socket(fd);
    return result ? result : canhost_stop ? 130 : 0;
}

static int register_access(const canhost_options_t *o)
{
    int fd = open_interface(o);
    j1939_reg_result_t reply;
    if (fd < 0) return 1;
    int write = o->command == CMD_REG_WRITE;
    int got = write ? j1939_reg_write(fd, (uint8_t)o->node, o->source, o->address, o->value, o->timeout_ms, &reply)
                    : j1939_reg_read(fd, (uint8_t)o->node, o->source, o->address, o->timeout_ms, &reply);
    can_close_socket(fd);
    if (canhost_stop) return 130;
    if (got < 0) { fputs("Error: register transport failed.\n", stderr); return 1; }
    if (!got) { fputs("Error: register response timed out.\n", stderr); return 1; }
    if (reply.status) { fprintf(stderr, "Error: device rejected register request (status %u).\n", reply.status); return 1; }
    if (write && reply.value != o->value) { fputs("Error: register write echo differs from request.\n", stderr); return 1; }
    printf("node=%d address=0x%04X value=%u (0x%08X)\n", o->node, o->address, reply.value, reply.value);
    if (write) fputs("Write acknowledged; saving/restarting remain explicit register operations.\n", stderr);
    return ferror(stdout) ? 1 : 0;
}

static int sync_frames(const canhost_options_t *o)
{
    int fd = open_interface(o), result = 0;
    uint64_t sent = 0, next = utils_now_ms();
    if (fd < 0) return 1;
    while (!canhost_stop && (!o->count || sent < o->count)) {
        uint64_t now = utils_now_ms();
        if (now >= next) {
            hipnuc_can_frame_t frame;
            hipnuc_j1939_build_trigger((uint8_t)o->node, o->source, o->pgn, &frame);
            if (can_send_frame(fd, &frame) < 0) { fputs("Error: trigger send failed.\n", stderr); result = 1; break; }
            ++sent;
            next = now + o->interval_ms; // do not burst to catch up after a host scheduling delay
        } else {
            uint64_t delay = next - now;
            utils_delay_ms((uint32_t)(delay < 20 ? delay : 20));
        }
    }
    fprintf(stderr, "Sent %llu trigger requests; triggers have no acknowledgement.\n", (unsigned long long)sent);
    can_close_socket(fd);
    return result ? result : canhost_stop ? 130 : 0;
}

int canhost_run(const canhost_options_t *o)
{
    switch (o->command) {
    case CMD_LIST: return list_interfaces();
    case CMD_SCAN: return scan(o);
    case CMD_READ: return read_samples(o);
    case CMD_REG_READ: case CMD_REG_WRITE: return register_access(o);
    case CMD_SYNC: return sync_frames(o);
    case CMD_UPDATE: return canhost_update(o);
    }
    return 2;
}
