#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include "global_options.h"
#include "serial_port.h"
#include "log.h"

#define CMD_REPLAY_TIMEOUT_MS (100)

static int execute_commands_from_file(int fd, const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        log_error("Unable to open file %s", filename);
        return -1;
    }

    char line[256];
    uint8_t recv_buf[2048];
    int line_number = 0;

    while (fgets(line, sizeof(line), file)) {
        line_number++;
        // Remove leading whitespace
        char *trimmed_line = line;
        while (isspace(*trimmed_line)) trimmed_line++;

        // Remove newline character from the end of the line
        trimmed_line[strcspn(trimmed_line, "\n")] = 0;
        
        // Skip empty lines and comment lines (starting with # or ;)
        if (trimmed_line[0] == '\0' || trimmed_line[0] == '#' || trimmed_line[0] == ';') {
            continue;
        }

        log_info("Send command (line %d): %s", line_number, trimmed_line);

        // Prepare command with CRLF
        char command_with_crlf[strlen(trimmed_line) + 3];
        snprintf(command_with_crlf, sizeof(command_with_crlf), "%s\r\n", trimmed_line);

        // Send command and receive response
        int len = serial_send_then_recv_str(fd, command_with_crlf, "OK", recv_buf, sizeof(recv_buf), CMD_REPLAY_TIMEOUT_MS);
        if (len > 0) {
            log_info("Received %d bytes:", len);
            for (int i = 0; i < len; i++) {
                if (recv_buf[i] == '\r' || recv_buf[i] == '\n') {
                    printf("%c", recv_buf[i]);
                } else {
                    printf("%c", isprint(recv_buf[i]) ? recv_buf[i] : '.');
                }
            }
            printf("\n");
        } else {
            log_info("No response or error.");
        }

    }

    fclose(file);
    return 0;
}

int cmd_write(GlobalOptions *opts, int argc, char *argv[]) {
    int fd = -1;

    // Check if there are enough arguments
    if (argc < 1) {
        log_error("Usage: write <COMMAND> or write <config_file>");
        return -1;
    }

    if ((fd = serial_port_open(opts->port_name)) < 0 || serial_port_configure(fd, opts->baud_rate) < 0) {
        log_error("Failed to open or configure port %s with %d", opts->port_name, opts->baud_rate);
        return -1;
    }

    uint8_t recv_buf[2048];
    serial_send_then_recv_str(fd, "LOG DISABLE\r\n", "OK", recv_buf, sizeof(recv_buf), CMD_REPLAY_TIMEOUT_MS);

    int result = 0;

    // Check if argv[0] is a file
    FILE *test = fopen(argv[0], "r");
    if (test) {
        fclose(test);
        result = execute_commands_from_file(fd, argv[0]);
    } else {
        // Otherwise, treat all arguments as a single command
        char command[256] = {0};
        for (int i = 0; i < argc; i++) {
            if (i > 0) strcat(command, " ");
            strcat(command, argv[i]);
        }

        log_info("Sending command: %s", command);

        char command_with_crlf[strlen(command) + 3];
        snprintf(command_with_crlf, sizeof(command_with_crlf), "%s\r\n", command);

        int len = serial_send_then_recv_str(fd, command_with_crlf, "OK", recv_buf, sizeof(recv_buf), CMD_REPLAY_TIMEOUT_MS);
        if (len > 0) {
            log_info("\nReceived %d bytes:", len);
            for (int i = 0; i < len; i++) {
                if (recv_buf[i] == '\r' || recv_buf[i] == '\n') {
                    printf("%c", recv_buf[i]);
                } else {
                    printf("%c", isprint(recv_buf[i]) ? recv_buf[i] : '.');
                }
            }
            printf("\n");  // Ensure output ends with a newline
        } else {
            log_info("No response or error. try: sudo ./hihost probe");
        }
    }

    serial_port_close(fd);
    return result;
}