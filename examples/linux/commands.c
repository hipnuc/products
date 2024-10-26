#include <ctype.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "commands.h"
#include "example_data.h"
#include "hipnuc_dec.h"
#include "nmea_dec.h"
#include "serial_port.h"
#include "hex2bin.h"
#include "kboot.h"
#include "log.h"

#define CMD_REPLAY_TIMEOUT_MS (100)
#define DISPLAY_UPDATE_INTERVAL 0.05
#define MAX_ATTEMPTS 2



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
static int cmd_probe(GlobalOptions *opts, int argc, char *argv[]);
static int cmd_update(GlobalOptions *opts, int argc, char *argv[]);

static int safe_sleep(unsigned long usec) {
    struct timespec ts;
    ts.tv_sec = usec / 1000000;
    ts.tv_nsec = (usec % 1000000) * 1000;
    
    while (nanosleep(&ts, &ts) == -1) {
        if (errno != EINTR) {
            return -1;
        }
    }
    return 0;
}

// Array of available commands
static command_t commands[] = {
    {"list", cmd_list},
    {"probe", cmd_probe},
    {"read", cmd_read},
    {"write", cmd_write},
    {"update", cmd_update},
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
    PortInfo *ports = NULL;
    int count = list_serial_ports(&ports);

    if (count < 0) {
        log_error("Failed to list serial ports.");
        return -1;
    }

    if (count == 0) {
        log_info("No serial ports found.");
        return 0;
    }

    print_port_list(ports, count);
    free_port_list(ports);
    return 0;
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
    uint8_t recv_buf[1024];
    char log_buf[512];
    hipnuc_raw_t hipnuc_raw = {0};
    nmea_raw_t nmea_raw = {0};
    struct timespec last_time, current_time, last_display_time;
    long long frame_count = 0;
    int frame_rate = 0;
    double elapsed_time = 0.0;

    if ((fd = serial_port_open(opts->port_name)) < 0 || serial_port_configure(fd, opts->baud_rate) < 0) {
        log_error("Failed to open or configure port %s with %d", opts->port_name, opts->baud_rate);
        return -1;
    }

    log_info("Reading from port %s at %d baud. Press CTRL+C to exit.", opts->port_name, opts->baud_rate);

    // Enable data output
    serial_send_then_recv_str(fd, "AT+EOUT=1\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);

    clock_gettime(CLOCK_MONOTONIC, &last_time);
    last_display_time = last_time;

    // Main reading loop
    while (1) {
        bool new_hipnuc_data = false;
        bool new_nmea_data = false;

        // Read data from serial port
        int len = serial_port_read(fd, (char *)recv_buf, sizeof(recv_buf));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                // Process HipNuc data
                if (hipnuc_input(&hipnuc_raw, recv_buf[i]) > 0) {
                    new_hipnuc_data = true;
                    frame_count++;
                }
                
                // Process NMEA data
                if (input_nmea(&nmea_raw, recv_buf[i]) > 0) {
                    new_nmea_data = true;
                    frame_count++;
                }
            }
        }

        // Calculate elapsed time
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        elapsed_time = (current_time.tv_sec - last_time.tv_sec) +
                       (current_time.tv_nsec - last_time.tv_nsec) / 1e9;
        double display_elapsed_time = (current_time.tv_sec - last_display_time.tv_sec) +
                                      (current_time.tv_nsec - last_display_time.tv_nsec) / 1e9;

        // Update display if new data is available and enough time has passed
        if ((new_hipnuc_data || new_nmea_data) && display_elapsed_time >= DISPLAY_UPDATE_INTERVAL) {
            // Clear screen
            printf("\033[H\033[J");

            // Display HipNuc data if available
            if (new_hipnuc_data) {
                hipnuc_dump_packet(&hipnuc_raw, log_buf, sizeof(log_buf));
                printf("%s\n", log_buf);
            }

            // Display NMEA data if available
            if (new_nmea_data) {
                nmea_dec_dump_msg(&nmea_raw, log_buf, sizeof(log_buf));
                printf("%s\n", log_buf);
            }

            printf("Frame Rate: %d fps\n", frame_rate);

            last_display_time = current_time;
        }

        // Update frame rate every second
        if (elapsed_time >= 1.0) {
            frame_rate = (int)(frame_count / elapsed_time);
            frame_count = 0;
            last_time = current_time;
        }

        // Short sleep to prevent CPU overuse
        safe_sleep(1000);  // 1ms sleep
    }

    serial_port_close(fd);
    return 0;
}



/**
 * @brief Reads and executes commands from a file.
 *
 * This function reads commands from the specified file and sends them to the connected serial device.
 * It reads each line from the file, skipping empty lines and comment lines, and sends the command
 * to the serial device. It then waits for the "OK" response from the device and logs the received
 * response. A short delay is added between commands to prevent overloading the serial device.
 *
 * @param fd The file descriptor of the open serial port.
 * @param filename The path to the file containing the commands to execute.
 * @return 0 on success, -1 on failure.
 */
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


/**
 * @brief Writes a command or executes commands from a file to the connected serial device.
 * 
 * This function is used to send a single command or execute a set of commands from a file
 * to the connected serial device. It first opens the serial port, configures it with the
 * specified baud rate, and then sends the command(s) to the device. The function waits for
 * the "OK" response from the device and logs the received response.
 * 
 * @param opts Global options structure containing the serial port name and baud rate.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments, where the first argument is either a single
 *             command or the path to a file containing a set of commands.
 * @return int 0 on success, -1 on failure.
 */
static int cmd_write(GlobalOptions *opts, int argc, char *argv[]) {
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

    // Disable data output before sending command
    uint8_t recv_buf[2048];
    serial_send_then_recv_str(fd, "AT+EOUT=0\r\n", "OK", recv_buf, sizeof(recv_buf), CMD_REPLAY_TIMEOUT_MS);

    int result = 0;

    if (access(argv[0], F_OK) != -1) {
        // If the argument is an existing file, execute commands from the file
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
            log_info("No response or error.");
        }
    }

    serial_port_close(fd);
    return result;
}


static int save_device_info(const char *port, int baud) {
    FILE *tmp_file = fopen(TMP_CONFIG_FILE, "w");
    if (!tmp_file) {
        return -1;
    }
    fprintf(tmp_file, "%s %d", port, baud);
    fclose(tmp_file);
    return 0;
}

static int load_device_info(char **port, int *baud) {
    FILE *tmp_file = fopen(TMP_CONFIG_FILE, "r");
    if (!tmp_file) {
        return -1;
    }
    char port_buf[256];
    int result = fscanf(tmp_file, "%255s %d", port_buf, baud);
    fclose(tmp_file);
    if (result == 2) {
        *port = strdup(port_buf);
        return 0;
    }
    return -1;
}

/**
 * @brief Probe for a connected device by iterating through available serial ports and baud rates.
 * 
 * This function attempts to find a connected device by opening each available serial port and
 * testing a set of common baud rates. If a device responds with the expected "OK" response,
 * the function logs a success message and returns 0. If no device is found, the function
 * returns -1.
 * 
 * @param opts Global options structure (unused in this function)
 * @param argc Number of command arguments (unused in this function)
 * @param argv Array of command arguments (unused in this function)
 * @return int 0 if a device was found, -1 otherwise
 */
static int cmd_probe(GlobalOptions *opts, int argc, char *argv[]) {
    PortInfo *ports;
    int len = 0;
    uint8_t recv_buf[2048];

    int port_count = list_serial_ports(&ports);
    if (port_count <= 0) {
        log_error("No serial ports found or failed to list serial ports.");
        return -1;
    }

    int baud_rates[] = {115200, 921600, 9600, 460800};
    int num_baud_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);

    bool device_found = false;
    char found_port[256] = {0};
    int found_baud = 0;
    char device_info[1024] = {0};

    for (int port_index = 0; port_index < port_count && !device_found; port_index++) {
        for (int i = 0; i < num_baud_rates && !device_found; i++) {
            int fd = serial_port_open(ports[port_index].name);
            if (fd < 0) {
                log_info("Failed to open port %s", ports[port_index].name);
                continue;
            }

            if (serial_port_configure(fd, baud_rates[i]) < 0) {
                log_info("Failed to configure port %s at %d baud", ports[port_index].name, baud_rates[i]);
                serial_port_close(fd);
                continue;
            }

            log_info("Probing %s at %d baud...", ports[port_index].name, baud_rates[i]);

            for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
                
                serial_send_then_recv_str(fd, "AT+EOUT=0\r\n", "OK", recv_buf, sizeof(recv_buf), 20);

                len = serial_send_then_recv_str(fd, "AT+INFO\r\n", "OK", recv_buf, sizeof(recv_buf), CMD_REPLAY_TIMEOUT_MS);

                if (len > 0 && strstr((char *)recv_buf, "OK") != NULL) {
                    log_info("Device found on %s at %d baud! (Attempt %d)", ports[port_index].name, baud_rates[i], attempt + 1);
                    device_found = true;
                    strncpy(found_port, ports[port_index].name, sizeof(found_port) - 1);
                    found_baud = baud_rates[i];
                    strncpy(device_info, (char *)recv_buf, sizeof(device_info) - 1);

                    serial_send_then_recv_str(fd, "AT+EOUT=1\r\n", "OK", recv_buf, sizeof(recv_buf), 10);
                    break;
                }
                
                if (attempt < MAX_ATTEMPTS - 1) {
                    safe_sleep(1 * 1000);
                }
            }

            serial_port_close(fd);
            
            if (device_found) break;
        }
    }

    free_port_list(ports);

    if (device_found) {
        printf("\n========== Device Found ==========\n");
        printf("Port: %s\n", found_port);
        printf("Baud Rate: %d\n", found_baud);
        printf("Device Info:\n%s\n", device_info);
        printf("\nExample commands to use this device:\n");
        printf("1. Read data:\n   sudo ./hihost -p %s -b %d read\n", found_port, found_baud);
        printf("2. Send a command:\n   sudo ./hihost -p %s -b %d write \"AT+INFO\"\n", found_port, found_baud);
        printf("===================================\n");

        save_device_info(found_port, found_baud);
        return 0;
    } else {
        log_error("No compatible device found on any port.");
        return -1;
    }
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



#define MAX_PING_ATTEMPTS 10
#define BAR_WIDTH 50

static int cmd_update(GlobalOptions *opts, int argc, char *argv[]) {
    if (argc != 1) {
        log_error("Usage: update <hex_file>");
        return -1;
    }

    int fd = -1;
    int last_percent = -1;
    const char *hex_file = argv[0];
    uint8_t recv_buf[2048];
    binary_data_t *bin_data = NULL;
    kboot_handle_t handle = {0};
    int result = -1;
    bool ping_success = false;

    // Convert HEX to BIN in memory
    bin_data = hex2bin_convert(hex_file);
    if (!bin_data) {
        log_error("HEX to BIN conversion failed");
        goto cleanup;
    }

    log_info("%s convert to binary size: %zu bytes", hex_file, bin_data->size);

    if ((fd = serial_port_open(opts->port_name)) < 0 || serial_port_configure(fd, opts->baud_rate) < 0) {
        log_error("Failed to open or configure port %s at %d baud", opts->port_name, opts->baud_rate);
        goto cleanup;
    }
    
    // Send reset command
    serial_send_then_recv_str(fd, "AT+RST\r\n", "OK", recv_buf, sizeof(recv_buf), 10);

    safe_sleep(10*1000);  // 10ms delay

    kboot_init(&handle, fd);
    kboot_set_debug(&handle, 0);

    // Attempt to ping multiple times
    for (int attempt = 0; attempt < MAX_PING_ATTEMPTS; attempt++) {
        if (kboot_ping(&handle)) {
            ping_success = true;
            break;
        }
    }

    if (!ping_success) {
        log_error("Failed to receive ping response after %d attempts", MAX_PING_ATTEMPTS);
        goto cleanup;
    }

    // Get properties
    if (!kboot_get_property(&handle, 0x0B, &handle.max_packet_size) ||
        !kboot_get_property(&handle, 0x04, &handle.flash_size) ||
        !kboot_get_property(&handle, 0x10, &handle.bl_version) ||
        !kboot_get_property(&handle, 0x39, &handle.app_version) ||
        !kboot_get_property(&handle, 0x05, &handle.flash_sector_size)) {
        log_error("Failed to get device properties");
        kboot_reset(&handle);
        goto cleanup;
    }

    log_info("Bootloader Connected!, Version: %d", handle.bl_version);

    if (!kboot_flash_erase_region(&handle, bin_data->start_address, bin_data->size)) {
        log_error("Failed to erase flash region");
        kboot_reset(&handle);
        goto cleanup;
    }

    if (!kboot_flash_write_memory(&handle, bin_data->start_address, bin_data->size)) {
        log_error("Failed to write memory");
        kboot_reset(&handle);
        goto cleanup;
    }

    uint32_t bytes_written = 0;
    uint32_t packet_size = handle.max_packet_size;
    while (bytes_written < bin_data->size) {
        uint32_t chunk_size = (bin_data->size - bytes_written) < packet_size ? (bin_data->size - bytes_written) : packet_size;

        if (!kboot_send_data(&handle, bin_data->data + bytes_written, chunk_size)) {
            log_error("Failed to send data packet");
            kboot_reset(&handle);
            goto cleanup;
        }

        bytes_written += chunk_size;

        int percent = (bytes_written * 100) / bin_data->size;
        if (percent != last_percent) {
            last_percent = percent;
            int filled_width = BAR_WIDTH * percent / 100;

            printf("\rWriting: [");
            for (int i = 0; i < BAR_WIDTH; ++i) {
                printf(i < filled_width ? "#" : " ");
            }
            printf("] %3d%%", percent);
            fflush(stdout);
        }
    }
    printf("\n");  // New line after progress bar

    safe_sleep(20*1000);

    if (!kboot_reset(&handle)) {
        log_error("Failed to reset device");
        goto cleanup;
    }

    log_info("Firmware Update successfully!");
    log_info("Re-power the board to boot the new firmware.");

    result = 0;  // Success

cleanup:
    if (bin_data) {
        free(bin_data->data);
        free(bin_data);
    }
    if (fd >= 0) {
        serial_port_close(fd);
    }
    return result;
}









