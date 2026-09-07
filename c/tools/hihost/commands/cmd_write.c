/* write COMMAND... | write FILE: send ASCII commands and print the reply. */
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "global_options.h"
#include "log.h"
#include "serial_port.h"

#define CMD_REPLY_TIMEOUT_MS 100
#define MAX_COMMAND_LEN 256

static void print_reply(const char *buf, int len)
{
    for (int i = 0; i < len; i++) {
        unsigned char c = (unsigned char)buf[i];
        putchar((c == '\r' || c == '\n' || isprint(c)) ? c : '.');
    }
    if (len == 0 || buf[len - 1] != '\n') {
        putchar('\n');
    }
}

/* Append "\r\n", send, print the reply. Returns 0 when something came back. */
static int send_command(int fd, const char *command)
{
    char line[MAX_COMMAND_LEN + 3];
    char recv_buf[2048];
    int n = snprintf(line, sizeof(line), "%s\r\n", command);
    if (n < 0 || (size_t)n >= sizeof(line)) {
        log_error("Command too long (max %d characters)", MAX_COMMAND_LEN);
        return -1;
    }
    int len = serial_send_then_recv_str(fd, line, NULL, recv_buf, sizeof(recv_buf), CMD_REPLY_TIMEOUT_MS);
    if (len <= 0) {
        log_info("No reply");
        return -1;
    }
    print_reply(recv_buf, len);
    return 0;
}

static int execute_commands_from_file(int fd, const char *filename)
{
    FILE *file = fopen(filename, "r");
    if (!file) {
        log_error("Cannot open %s", filename);
        return -1;
    }

    char line[MAX_COMMAND_LEN];
    int line_number = 0;
    while (fgets(line, sizeof(line), file)) {
        line_number++;
        char *p = line;
        while (isspace((unsigned char)*p)) p++;
        p[strcspn(p, "\r\n")] = '\0';
        if (*p == '\0' || *p == '#' || *p == ';') {
            continue;
        }
        log_info("line %d: %s", line_number, p);
        send_command(fd, p);
    }
    fclose(file);
    return 0;
}

int cmd_write(GlobalOptions *opts, int argc, char *argv[])
{
    if (argc < 1) {
        log_error("Usage: write COMMAND... | write FILE");
        return -1;
    }

    int fd = serial_port_open(opts->port_name);
    if (fd < 0 || serial_port_configure(fd, opts->baud_rate) < 0) {
        log_error("Cannot open %s at %d baud", opts->port_name, opts->baud_rate);
        if (fd >= 0) serial_port_close(fd);
        return -1;
    }

    char recv_buf[512];
    serial_send_then_recv_str(fd, "LOG DISABLE\r\n", NULL, recv_buf, sizeof(recv_buf), CMD_REPLY_TIMEOUT_MS);

    int result;
    FILE *test = fopen(argv[0], "r");
    if (test) {
        fclose(test);
        result = execute_commands_from_file(fd, argv[0]);
    } else {
        /* Join the arguments with single spaces into one command. */
        char command[MAX_COMMAND_LEN];
        size_t used = 0;
        result = 0;
        for (int i = 0; i < argc; i++) {
            int n = snprintf(command + used, sizeof(command) - used, "%s%s", i ? " " : "", argv[i]);
            if (n < 0 || (size_t)n >= sizeof(command) - used) {
                log_error("Command too long (max %d characters)", MAX_COMMAND_LEN - 1);
                result = -1;
                break;
            }
            used += (size_t)n;
        }
        if (result == 0) {
            log_info("Sending: %s", command);
            if (send_command(fd, command) != 0) {
                log_info("Check the port and baud rate, or run `hihost probe`");
                result = -1;
            }
        }
    }

    serial_port_close(fd);
    return result;
}
