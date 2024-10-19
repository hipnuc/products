#include <ctype.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <unistd.h>

#include "commands.h"
#include "example_data.h"
#include "hipnuc_dec.h"
#include "nmea_dec.h"
#include "serial_port.h"
#include "log.h"

#define REFRESH_INTERVAL_MS (50)
#define CMD_REPLAY_TIMEOUT_MS (300)

// Structure to hold command information
typedef struct {
    const char *name;  // Command name
    int (*func)(GlobalOptions*, int, char**);  // Function pointer to command handler
} command_t;

// Function prototypes for command handlers
static int cmd_list(GlobalOptions *opts, int argc, char *argv[]);
static int cmd_read(GlobalOptions *opts, int argc, char *argv[]);
static int cmd_write(GlobalOptions *opts, int argc, char *argv[]);
static int cmd_example(GlobalOptions *opts, int argc, char *argv[]);

// Array of available commands
static command_t commands[] = {
    {"list", cmd_list},
    {"read", cmd_read},
    {"write", cmd_write},
    {"example", cmd_example},
    {NULL, NULL}  // Sentinel to mark end of array
};

/**
 * @brief Execute the specified command
 * 
 * @param command_name Name of the command to execute
 * @param opts Global options structure
 * @param argc Number of command arguments
 * @param argv Array of command arguments
 * @return int 0 on success, -1 on failure
 */
int execute_command(const char *command_name, GlobalOptions *opts, int argc, char *argv[]) {
    for (int i = 0; commands[i].name != NULL; i++) {
        if (strcmp(command_name, commands[i].name) == 0) {
            return commands[i].func(opts, argc, argv);
        }
    }
    log_error("Unknown command: %s", command_name);
    return -1;
}

/**
 * @brief List available serial ports
 * 
 * @param opts Global options structure (unused in this function)
 * @param argc Number of command arguments (unused in this function)
 * @param argv Array of command arguments (unused in this function)
 * @return int 0 on success, -1 on failure
 */
static int cmd_list(GlobalOptions *opts, int argc, char *argv[]) {
    PortInfo *ports;
    int count = list_serial_ports(&ports);
    if (count > 0) {
        print_port_list(ports, count);
        free_port_list(ports);
        return 0;
    } else if (count == 0) {
        log_info("No serial ports found.");
        return 0;
    } else {
        log_error("Failed to list serial ports.");
        return -1;
    }
}

/**
 * @brief Read data from the specified serial port
 * 
 * @param opts Global options structure
 * @param argc Number of command arguments (unused in this function)
 * @param argv Array of command arguments (unused in this function)
 * @return int 0 on success, -1 on failure
 */
static int cmd_read(GlobalOptions *opts, int argc, char *argv[]) {
    int fd = -1;
    int result = 0;

    hipnuc_raw_t hipnuc_raw = {0};
    nmea_raw_t nmea_raw = {0};
    uint8_t recv_buf[2048];
    char log_buf[512];

    // Variables for frame rate calculation
    struct timespec last_time, current_time;
    long long frame_count = 0;
    int frame_rate = 0;
    double elapsed_time = 0.0;

    do {
        // Check if port name is provided
        if (!opts->port_name) {
            log_error("Port name is required for read command.");
            result = -1;
            break;
        }

        // Open the serial port
        fd = serial_port_open(opts->port_name);
        if (fd < 0) {
            log_error("Failed to open port %s", opts->port_name);
            result = -1;
            break;
        }

        // Configure the serial port
        if (serial_port_configure(fd, opts->baud_rate) < 0) {
            log_error("Failed to configure port %s", opts->port_name);
            result = -1;
            break;
        }

        log_info("Reading from port %s at %d baud, Press CTRL+C to exit.\n", opts->port_name, opts->baud_rate);

        // Enable data output
        serial_send_then_recv(fd, "AT+EOUT=1\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);

        clock_gettime(CLOCK_MONOTONIC, &last_time);

        // Main reading loop
        while (1) 
        {
            int len = serial_port_read(fd, (char *)recv_buf, sizeof(recv_buf));
            if (len > 0) 
            {
                for (int i = 0; i < len; i++) {
                    // Process HipNuc data
                    if (hipnuc_input(&hipnuc_raw, recv_buf[i])) {
                        frame_count++;
                        hipnuc_dump_packet(&hipnuc_raw, log_buf, sizeof(log_buf));
                        printf("\033[H\033[J%s", log_buf);  // Clear screen and print data
                        printf("\nFrame Rate: %d fps\n", frame_rate);
                    }

                    // Process NMEA data
                    if (input_nmea(&nmea_raw, recv_buf[i]) > 0) 
                    {
                        frame_count++;
                        nmea_dec_dump_msg(&nmea_raw, log_buf, sizeof(log_buf));
                        printf("\033[H\033[J%s", log_buf);  // Clear screen and print data
                        printf("\nFrame Rate: %d fps\n", frame_rate);
                    }
                }
            }

            // Calculate elapsed time with high precision
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            elapsed_time = (current_time.tv_sec - last_time.tv_sec) +
                           (current_time.tv_nsec - last_time.tv_nsec) / 1e9;

            // Update frame rate every second
            if (elapsed_time >= 1.0) {
                // Calculate frame rate as an integer
                frame_rate = (int)(frame_count / elapsed_time);

                // Reset counter and time
                frame_count = 0;
                last_time = current_time;
            }

            usleep(REFRESH_INTERVAL_MS*1000);
        }

    } while(0);

    // Close the serial port if it was opened
    if (fd >= 0) {
        serial_port_close(fd);
    }

    return result;
}

// Read and execute commands from a file
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
        // Remove newline character from the end of the line
        line[strcspn(line, "\n")] = 0;
        
        // Skip empty lines and comment lines
        if (line[0] == '\0' || line[0] == '#') {
            continue;
        }

        log_info("Send command (line %d): %s", line_number, line);

        // Prepare command with CRLF
        char command_with_crlf[strlen(line) + 3];
        snprintf(command_with_crlf, sizeof(command_with_crlf), "%s\r\n", line);

        // Send command and receive response
        int len = serial_send_then_recv(fd, command_with_crlf, "OK", recv_buf, sizeof(recv_buf), CMD_REPLAY_TIMEOUT_MS);
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

        // Add a short delay between commands
        usleep(10000);
    }

    fclose(file);
    return 0;
}

static int cmd_write(GlobalOptions *opts, int argc, char *argv[]) {

    if (!opts->port_name || argc < 1) {
        log_error("Usage: write <COMMAND> or write <config_file>");
        return -1;
    }

    int fd = serial_port_open(opts->port_name);
    if (fd < 0) {
        log_error("Failed to open port %s", opts->port_name);
        return -1;
    }

    if (serial_port_configure(fd, opts->baud_rate) < 0) {
        log_error("Failed to configure port %s", opts->port_name);
        serial_port_close(fd);
        return -1;
    }

    // Disable data output before sending command
    uint8_t recv_buf[2048];
    serial_send_then_recv(fd, "AT+EOUT=0\r\n", "OK", recv_buf, sizeof(recv_buf), CMD_REPLAY_TIMEOUT_MS);

    int result = 0;

    if (argc == 1 && access(argv[0], F_OK) != -1) {
        // If the argument is a file that exists, execute commands from the file
        result = execute_commands_from_file(fd, argv[0]);
    } else {
        // Otherwise, treat it as a single command
        char command[256] = {0};
        for (int i = 0; i < argc; i++) {
            if (i > 0) strcat(command, " ");
            strcat(command, argv[i]);
        }

        log_info("Sending command: %s", command);

        char command_with_crlf[strlen(command) + 3];
        snprintf(command_with_crlf, sizeof(command_with_crlf), "%s\r\n", command);

        int len = serial_send_then_recv(fd, command_with_crlf, "OK", recv_buf, sizeof(recv_buf), CMD_REPLAY_TIMEOUT_MS);
        if (len > 0) {
            log_info("\nReceived %d bytes:", len);
            for (int i = 0; i < len; i++) {
                if (recv_buf[i] == '\r' || recv_buf[i] == '\n') {
                    printf("%c", recv_buf[i]);
                } else {
                    printf("%c", isprint(recv_buf[i]) ? recv_buf[i] : '.');
                }
            }
        } else {
            log_info("No response or error.");
        }
    }

    serial_port_close(fd);
    return result;
}


/**
 * @brief Process example data (for demonstration purposes)
 * 
 * @param opts Global options structure (unused in this function)
 * @param argc Number of command arguments (unused in this function)
 * @param argv Array of command arguments (unused in this function)
 * @return int Result of processing example data
 */
static int cmd_example(GlobalOptions *opts, int argc, char *argv[]) {
    return process_example_data();
}
