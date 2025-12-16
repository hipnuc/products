#include <unistd.h>
#include <linux/can.h>
#include <stdint.h>
#include "utils.h"
#include "can_interface.h"
#include "hipnuc_can_common.h"
#include "j1939_reg_api.h"

static int j1939_send(int fd, const hipnuc_can_frame_t *hip)
{
    struct can_frame f;
    utils_hipnuc_can_to_linux_can(hip, &f);
    ssize_t w = write(fd, &f, sizeof(f));
    return (w == sizeof(f)) ? 0 : -1;
}

static int j1939_await(int fd, uint16_t expect_addr, hipnuc_j1939_cmd_t expect_cmd, j1939_reg_result_t *out, int timeout_ms)
{
    uint64_t deadline = (uint64_t)utils_get_timestamp_ms() + (uint64_t)timeout_ms;
    while ((int64_t)((uint64_t)utils_get_timestamp_ms() - deadline) <= 0) {
        struct can_frame f;
        uint64_t hw_ts_us = 0;
        int r = can_receive_frame_ts(fd, &f, &hw_ts_us);
        if (r < 0) return -1;
        if (r == 0) continue;
        hipnuc_can_frame_t hf;
        utils_linux_can_to_hipnuc_can(&f, hw_ts_us, &hf);
        if (!hipnuc_j1939_is_cfg_frame(&hf)) continue;
        uint16_t addr = 0; hipnuc_j1939_cmd_t cmd = 0; uint8_t status = 0; uint32_t val = 0;
        if (hipnuc_j1939_parse_cmd(&hf, &addr, &cmd, &status, &val) == 0) {
            if (addr == expect_addr && cmd == expect_cmd) {
                if (out) { out->value = val; out->status = status; }
                return 1;
            }
        }
    }
    return 0;
}

int j1939_reg_read(int fd, uint8_t da, uint8_t sa, uint16_t addr, int timeout_ms, j1939_reg_result_t *out)
{
    hipnuc_can_frame_t cfg;
    hipnuc_j1939_build_reg_read_cmd(da, sa, addr, &cfg);
    if (j1939_send(fd, &cfg) < 0) return -1;
    return j1939_await(fd, addr, HIPNUC_J1939_CMD_READ, out, timeout_ms);
}

int j1939_reg_write(int fd, uint8_t da, uint8_t sa, uint16_t addr, uint32_t val, int timeout_ms, j1939_reg_result_t *out)
{
    hipnuc_can_frame_t cfg;
    hipnuc_j1939_build_reg_write_cmd(da, sa, addr, val, &cfg);
    if (j1939_send(fd, &cfg) < 0) return -1;
    return j1939_await(fd, addr, HIPNUC_J1939_CMD_WRITE, out, timeout_ms);
}

