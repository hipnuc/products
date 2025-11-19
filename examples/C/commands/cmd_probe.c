#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "global_options.h"
#include "serial_port.h"
#include "log.h"
#include "cmd_utils.h"

#define CMD_REPLAY_TIMEOUT_MS (300)
#define MAX_ATTEMPTS 2

static int save_device_info(const char *port, int baud) {
    return ini_update_serial(HIHOST_INI_ABS_PATH, port, baud);
}

int cmd_probe(GlobalOptions *opts, int argc, char *argv[]) {
    (void)opts; (void)argc; (void)argv;
    PortInfo *ports;
    int len = 0;
    uint8_t recv_buf[2048];

    int port_count = list_serial_ports(&ports);
    if (port_count <= 0) {
        log_error("No serial ports found or failed to list serial ports.");
        return -1;
    }

    int baud_rates[] = {115200, 921600, 9600, 460800, 230400, 256000};
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
                for (int k = 0; k < 3; k++) {
                    serial_send_then_recv_str(fd, "LOG DISABLE\r\n", "OK", recv_buf, sizeof(recv_buf), 200);
                    safe_sleep(50);
                }
                len = serial_send_then_recv_str(fd, "AT+INFO\r\n", "OK", recv_buf, sizeof(recv_buf), CMD_REPLAY_TIMEOUT_MS);
                if (len > 0 && strstr((char *)recv_buf, "OK") != NULL) {
                    log_info("Device found on %s at %d baud! (Attempt %d)", ports[port_index].name, baud_rates[i], attempt + 1);
                    device_found = true;
                    strncpy(found_port, ports[port_index].name, sizeof(found_port) - 1);
                    found_baud = baud_rates[i];
                    strncpy(device_info, (char *)recv_buf, sizeof(device_info) - 1);

                    serial_send_then_recv_str(fd, "LOG ENABLE\r\n", "OK", recv_buf, sizeof(recv_buf), 10);
                    break;
                }
                if (attempt < MAX_ATTEMPTS - 1) {
                    safe_sleep(300);
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