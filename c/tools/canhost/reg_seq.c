#include "reg_seq.h"

#include <stdio.h>

#include "j1939_reg_api.h"
#include "log.h"
#include "utils.h"

int reg_seq_execute(int fd, uint8_t da, uint8_t sa, const reg_seq_cmd_t *cmd)
{
    if (!cmd || !cmd->steps) {
        log_error("Invalid command sequence");
        return -1;
    }

    log_info("Executing sequence: %s (%s)", cmd->name, cmd->description);

    for (size_t i = 0; i < cmd->step_count; i++) {
        const reg_seq_step_t *step = &cmd->steps[i];
        j1939_reg_result_t res = { 0, 0 };
        int ret;

        switch (step->type) {
        case REG_SEQ_WRITE: {
            int timeout = step->timeout_ms > 0 ? step->timeout_ms : 200;
            log_debug("  Step %zu: WRITE 0x%04X = 0x%08X", i, step->addr, step->val);
            ret = j1939_reg_write(fd, da, sa, step->addr, step->val, timeout, &res);
            if (ret <= 0) {
                log_error("  No write reply for register 0x%04X", step->addr);
                return -1;
            }
            if (res.status != 0) {
                log_error("  Write returned error status %u", res.status);
                return -1;
            }
            break;
        }

        case REG_SEQ_POLL: {
            uint64_t start = utils_now_ms();
            int success = 0;
            int have_value = 0;

            log_debug("  Step %zu: POLL 0x%04X until 0x%08X (%d ms)",
                      i, step->addr, step->val, step->timeout_ms);
            while (utils_now_ms() - start < (uint64_t)step->timeout_ms) {
                ret = j1939_reg_read(fd, da, sa, step->addr, 100, &res);
                if (ret > 0 && res.status == 0) {
                    have_value = 1;
                    if (res.value == step->val) {
                        success = 1;
                        break;
                    }
                    printf("    [Node %u] 0x%04X = %u\r", da, step->addr, res.value);
                    fflush(stdout);
                }
                utils_delay_ms(100);
            }
            printf("\n");
            if (!success) {
                if (have_value) {
                    log_error("  Polling timed out, last value 0x%08X", res.value);
                } else {
                    log_error("  Polling timed out, no reply from node %u", da);
                }
                return -1;
            }
            break;
        }

        case REG_SEQ_DELAY:
            log_debug("  Step %zu: DELAY %d ms", i, step->timeout_ms);
            utils_delay_ms((uint32_t)step->timeout_ms);
            break;

        case REG_SEQ_READ:
            log_debug("  Step %zu: READ 0x%04X", i, step->addr);
            ret = j1939_reg_read(fd, da, sa, step->addr, 200, &res);
            if (ret <= 0) {
                log_error("  No read reply for register 0x%04X", step->addr);
                return -1;
            }
            if (res.status != 0) {
                log_error("  Read returned error status %u", res.status);
                return -1;
            }
            printf("[Node %u] Reg 0x%04X: 0x%08X (%u)\n", da, step->addr, res.value, res.value);
            break;

        default:
            log_error("  Unknown step type %d", (int)step->type);
            return -1;
        }
    }

    log_debug("Sequence %s completed", cmd->name);
    return 0;
}
