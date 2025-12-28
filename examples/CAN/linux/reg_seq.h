#ifndef REG_SEQ_H
#define REG_SEQ_H

#include <stdint.h>
#include <stddef.h>

/**
 * Type of step in a register operation sequence.
 */
typedef enum {
    REG_SEQ_WRITE,      // Write a value to a register
    REG_SEQ_POLL,       // Read a register repeatedly until it matches a condition
    REG_SEQ_DELAY,      // Wait for a specified time
    REG_SEQ_READ        // Read and print a register value
} reg_seq_type_t;

/**
 * Definition of a single step.
 */
typedef struct {
    reg_seq_type_t type;
    uint16_t addr;      // Register address (for WRITE and POLL)
    uint32_t val;       // For WRITE: value to write. For POLL: expected value.
    int timeout_ms;     // For POLL: max time to wait. For DELAY: duration in ms.
} reg_seq_step_t;

/**
 * A named sequence of steps.
 */
typedef struct {
    const char *name;           // Command name (e.g., "reset")
    const char *description;    // Description for help
    const reg_seq_step_t *steps;
    size_t step_count;
} reg_seq_cmd_t;

/**
 * Execute a sequence of register operations.
 *
 * @param fd Open CAN socket file descriptor.
 * @param da Destination Address (target node).
 * @param sa Source Address (host).
 * @param cmd The command sequence to execute.
 * @return 0 on success, -1 on failure.
 */
int reg_seq_execute(int fd, uint8_t da, uint8_t sa, const reg_seq_cmd_t *cmd);

#endif // REG_SEQ_H
