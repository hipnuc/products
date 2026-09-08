#include "j1939_reg_api.h"

#include <stdint.h>

#include "can_interface.h"
#include "hipnuc_j1939.h"
#include "utils.h"
#include "cli.h"

static int await_reply(int fd, uint8_t da, uint8_t sa, uint16_t expect_addr, hipnuc_j1939_cmd_t expect_cmd,
                       j1939_reg_result_t *out, int timeout_ms)
{
    uint64_t deadline = utils_now_ms() + (uint64_t)(timeout_ms > 0 ? timeout_ms : 0);

    while (!canhost_stop) {
        uint64_t now = utils_now_ms();
        if (now >= deadline) {
            break;
        }
        can_rx_frame_t rx;
        int remaining = (int)(deadline - now);
        int r = can_receive_frames(fd, &rx, 1, remaining < 50 ? remaining : 50);
        if (r < 0) {
            return -1;
        }
        if (r == 0) {
            continue;
        }

        uint8_t source = 0, status = 0;
        uint16_t addr = 0;
        uint32_t value = 0;
        hipnuc_j1939_cmd_t cmd = HIPNUC_J1939_CMD_READ;
        if (hipnuc_j1939_parse_config(&rx.frame, &source, &addr, &cmd, &status, &value) != 0) {
            continue;
        }
        /* Only the reply of the addressed node counts; several devices may
         * answer on one bus. */
        if (source != da || ((rx.frame.id >> 8) & 0xff) != sa ||
            addr != expect_addr || cmd != expect_cmd) {
            continue;
        }
        if (out) {
            out->value = value;
            out->status = status;
        }
        return 1;
    }
    return 0;
}

int j1939_reg_read(int fd, uint8_t da, uint8_t sa, uint16_t addr, int timeout_ms, j1939_reg_result_t *out)
{
    hipnuc_can_frame_t req;
    hipnuc_j1939_build_reg_read(da, sa, addr, &req);
    if (can_send_frame(fd, &req) < 0) {
        return -1;
    }
    return await_reply(fd, da, sa, addr, HIPNUC_J1939_CMD_READ, out, timeout_ms);
}

int j1939_reg_write(int fd, uint8_t da, uint8_t sa, uint16_t addr, uint32_t val, int timeout_ms, j1939_reg_result_t *out)
{
    hipnuc_can_frame_t req;
    hipnuc_j1939_build_reg_write(da, sa, addr, val, &req);
    if (can_send_frame(fd, &req) < 0) {
        return -1;
    }
    return await_reply(fd, da, sa, addr, HIPNUC_J1939_CMD_WRITE, out, timeout_ms);
}
