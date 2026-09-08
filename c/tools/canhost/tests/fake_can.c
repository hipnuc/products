/* SocketCAN test double. Only transport and time are replaced; CLI, parsers,
 * formatter, register matching and upgrade state machine are the real code. */
#include "can_interface.h"
#include "cli.h"
#include "utils.h"
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t clock_ms;
static int batch_sent, reply_ready;
static hipnuc_can_frame_t request;

static int scene(const char *name)
{
    const char *value = getenv("CANHOST_TEST_SCENE");
    return value && strcmp(value, name) == 0;
}
uint64_t utils_now_ms(void) { return clock_ms; }
void utils_delay_ms(uint32_t milliseconds) { clock_ms += milliseconds; }
int can_open_socket(const char *name)
{
    (void)name;
    fputs("FAKE_OPEN\n", stderr);
    if (scene("open_error")) { errno = ENODEV; return -1; }
    return 42;
}
void can_close_socket(int fd) { (void)fd; fputs("FAKE_CLOSE\n", stderr); }
int can_list_interfaces(can_interface_info_t *out, int maximum)
{
    (void)maximum;
    strcpy(out[0].name, "fake0");
    strcpy(out[0].state, "up");
    return 1;
}
static int goto_app(void)
{
    return !request.is_extended && request.data[0] == 0x23 && request.data[3] == 9;
}
int can_send_frame(int fd, const hipnuc_can_frame_t *frame)
{
    (void)fd;
    request = *frame;
    if (scene("send_error") || (goto_app() && scene("goto_send_error"))) return -1;
    reply_ready = 1;
    return 0;
}
int can_receive_frames(int fd, can_rx_frame_t *out, size_t maximum, int timeout_ms)
{
    (void)fd;
    clock_ms += timeout_ms > 0 ? (uint64_t)timeout_ms : 1;
    if (scene("receive_error") || (goto_app() && scene("goto_receive_error"))) return -1;
    if (scene("idle") || (goto_app() && scene("goto_timeout"))) return 0;
    if (reply_ready) {
        memset(out, 0, sizeof(*out));
        reply_ready = 0;
        out->frame = request;
        if (request.is_extended) {
            out->frame.id = 0x0cef5500 | ((request.id >> 8) & 255);
            if (request.data[2] == 3) {
                out->frame.data[4] = 42;
                out->frame.data[5] = out->frame.data[6] = out->frame.data[7] = 0;
            }
            if (scene("reg_error")) out->frame.data[3] = 2;
            if (scene("reg_echo_error")) out->frame.data[4] ^= 1;
            if (scene("reg_other_host")) out->frame.id ^= (1 << 8);
            if (scene("reg_other_node")) out->frame.id ^= 1;
        } else {
            out->frame.id = 0x580 + (request.id - 0x600);
            out->frame.data[0] = request.data[0] == 0x23 || request.data[0] == 0x21 ?
                                 0x60 : (0x20 | (request.data[0] & 0x10));
            if (goto_app() && scene("goto_abort")) out->frame.data[0] = 0x80;
            if (scene("update_interrupt")) raise(SIGINT);
        }
        return 1;
    }
    if (!batch_sent && !request.len) {
        static const uint8_t nodes[5] = {8, 250, 8, 255, 8};
        size_t count = maximum < 5 ? maximum : 5;
        for (size_t i = 0; i < count; ++i) {
            memset(&out[i], 0, sizeof(out[i]));
            out[i].frame.id = 0x0cff3400 | nodes[i];
            out[i].frame.is_extended = 1;
            out[i].frame.len = 8;
            out[i].frame.data[1] = 8;  /* 2048 * 9.8 / 2048 = 9.8 m/s^2 */
            out[i].timestamp_us = 1700000000000000ULL + i;
        }
        batch_sent = 1;
        if (scene("interrupt")) raise(SIGINT);
        return (int)count;
    }
    return 0;
}

// Exercise the real command's close-error path without a filesystem dependency.
int __real_fclose(FILE *file);
int __wrap_fclose(FILE *file)
{
    int result = __real_fclose(file);
    if (scene("close_error")) { errno = EIO; return EOF; }
    return result;
}
