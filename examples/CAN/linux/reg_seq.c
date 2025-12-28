#include "reg_seq.h"
#include "j1939_reg_api.h"
#include "log.h"
#include "utils.h"
#include <unistd.h>
#include <stdio.h>

int reg_seq_execute(int fd, uint8_t da, uint8_t sa, const reg_seq_cmd_t *cmd)
{
    if (!cmd || !cmd->steps) {
        log_error("Invalid command sequence");
        return -1;
    }

    log_info("Executing sequence: %s (%s)", cmd->name, cmd->description);

    for (size_t i = 0; i < cmd->step_count; i++) {
        const reg_seq_step_t *step = &cmd->steps[i];
        j1939_reg_result_t res;
        int ret;

        switch (step->type) {
            case REG_SEQ_WRITE:
                log_debug("  Step %zu: WRITE Addr 0x%04X = 0x%08X", i, step->addr, step->val);
                int timeout = (step->timeout_ms > 0) ? step->timeout_ms : 200;
                ret = j1939_reg_write(fd, da, sa, step->addr, step->val, timeout, &res); // Default timeout 200ms for write
                if (ret <= 0) {
                    log_error("  Failed to write register 0x%04X", step->addr);
                    return -1;
                }
                if (res.status != 0) {
                    log_error("  Write returned error status: %d", res.status);
                    return -1;
                }
                break;

            case REG_SEQ_POLL:
                log_debug("  Step %zu: POLL Addr 0x%04X Expect 0x%08X Timeout %dms", 
                          i, step->addr, step->val, step->timeout_ms);
                
                uint64_t start = utils_get_timestamp_ms();
                int success = 0;
                
                while (utils_get_timestamp_ms() - start < (uint64_t)step->timeout_ms) {
                    ret = j1939_reg_read(fd, da, sa, step->addr, 100, &res); // Single read timeout 100ms
                    if (ret > 0 && res.status == 0) {
                        if (res.value == step->val) {
                            success = 1;
                            break;
                        } else {
                            // Print progress if it changes? Or just wait silently.
                            // For mag calibration progress, user might want to see it.
                            // But let's keep it simple as requested: poll until match.
                            log_debug("    Poll 0x%04X: 0x%08X (Waiting for 0x%08X)", step->addr, res.value, step->val);
                            printf("    [Node %d] Progress: %u%%\r", da, res.value); // Show progress dynamically?
                            fflush(stdout);
                        }
                    }
                    utils_delay_ms(100); // Poll interval
                }
                printf("\n"); // Clear line after progress

                if (!success) {
                    log_error("  Polling timed out. Last value: 0x%08X", res.value);
                    return -1;
                }
                break;

            case REG_SEQ_DELAY:
                log_debug("  Step %zu: DELAY %d ms", i, step->timeout_ms);
                utils_delay_ms(step->timeout_ms);
                break;

            case REG_SEQ_READ:
                log_debug("  Step %zu: READ Addr 0x%04X", i, step->addr);
                ret = j1939_reg_read(fd, da, sa, step->addr, 200, &res); // Timeout 200ms
                if (ret <= 0) {
                    log_error("  Failed to read register 0x%04X", step->addr);
                    return -1;
                }
                if (res.status != 0) {
                    log_error("  Read returned error status: %d", res.status);
                    return -1;
                }
                printf("[Node %d] Reg 0x%04X: 0x%08X (%u)\n", da, step->addr, res.value, res.value);
                break;

            default:
                log_error("  Unknown step type: %d", step->type);
                return -1;
        }
    }

    log_debug("Sequence %s completed successfully", cmd->name);
    return 0;
}
