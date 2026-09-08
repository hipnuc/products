#include "cli.h"
#include "can_interface.h"
#include "hipnuc_can_update.h"
#include "hexfile.h"
#include "utils.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>

static int update_send(void *user, const hipnuc_can_frame_t *frame)
{
    if (canhost_stop) return -1;
    return can_send_frame(*(int *)user, frame);
}

static int update_wait(void *user, uint32_t id, hipnuc_can_frame_t *frame, uint32_t timeout_ms)
{
    uint64_t start = utils_now_ms();
    while (!canhost_stop) {
        uint64_t elapsed = utils_now_ms() - start;
        if (elapsed >= timeout_ms) return 1;  /* timeout is distinct from I/O failure */
        uint64_t remaining = timeout_ms - elapsed;
        can_rx_frame_t received;
        int count = can_receive_frames(*(int *)user, &received, 1, remaining < 50 ? (int)remaining : 50);
        if (count < 0) return -1;
        if (!count) continue;
        if (received.frame.is_extended || received.frame.is_remote || received.frame.is_error ||
            received.frame.id != id) continue;
        *frame = received.frame;
        return 0;
    }
    return -1;
}

static void update_delay(void *user, uint32_t milliseconds)
{
    (void)user;
    while (milliseconds && !canhost_stop) {
        uint32_t part = milliseconds < 20 ? milliseconds : 20;
        utils_delay_ms(part);
        milliseconds -= part;
    }
}

static void update_progress(void *user, uint8_t node, uint8_t percent)
{
    (void)user;
    fprintf(stderr, "\rNode %u: %u%%", (unsigned)node, (unsigned)percent);
    if (percent == 100) fputc('\n', stderr);
}

static void update_log(void *user, uint8_t node, const char *message)
{
    (void)user;
    fprintf(stderr, "Node %u: %s\n", (unsigned)node, message);
}

int canhost_update(const canhost_options_t *o)
{
    hex_image_t image;
    int loaded = o->raw_binary ? binfile_load(o->image_path, &image) : hexfile_load(o->image_path, &image);
    if (loaded < 0) return 1;
    if (canhost_stop) { hex_image_free(&image); return 130; }
    int fd = can_open_socket(o->interface);
    if (fd < 0) {
        fprintf(stderr, "Error: cannot open %s: %s\n", o->interface, strerror(errno));
        hex_image_free(&image);
        return 1;
    }
    fprintf(stderr, "Updating node %d on %s from %s (%u bytes).\n",
            o->node, o->interface, o->image_path, (unsigned)image.size);
    hipnuc_can_update_port_t port = {update_send, update_wait, update_delay, update_progress, update_log, &fd};
    hipnuc_can_update_ctx_t updater;
    hipnuc_can_update_init(&updater, &port);
    int status = hipnuc_can_update_node(&updater, (uint8_t)o->node, image.data, image.size);
    can_close_socket(fd);
    hex_image_free(&image);
    if (canhost_stop) {
        fputs("Update interrupted; application restart was not requested after failure.\n", stderr);
        return 130;
    }
    if (status != HIPNUC_CAN_UPDATE_OK) {
        fprintf(stderr, "Error: update failed: %s\n", hipnuc_can_update_strerror(status));
        return 1;
    }
    fputs("Image transfer acknowledged and application start requested; running firmware is not verified.\n", stderr);
    return 0;
}
