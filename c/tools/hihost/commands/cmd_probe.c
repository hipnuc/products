/* probe [--save]: try every serial port at every supported baud rate until
 * a device answers `LOG VERSION` with OK. */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "global_options.h"
#include "hihost_config.h"
#include "log.h"
#include "serial_port.h"

#define CMD_REPLY_TIMEOUT_MS 300
#define MAX_ATTEMPTS 2

int cmd_probe(GlobalOptions *opts, int argc, char *argv[])
{
    (void)opts;
    bool save = false;
    for (int i = 0; i < argc; ++i) {
        if (strcmp(argv[i], "--save") == 0) {
            save = true;
        } else {
            log_error("Usage: probe [--save]");
            return -1;
        }
    }

    PortInfo *ports = NULL;
    int port_count = list_serial_ports(&ports);
    if (port_count <= 0) {
        log_error("No serial ports found");
        free_port_list(ports);
        return -1;
    }

    bool device_found = false;
    char found_port[MAX_PORT_NAME] = {0};
    int found_baud = 0;
    char device_info[512] = {0};
    char recv_buf[512];

    for (int p = 0; p < port_count && !device_found; p++) {
        for (int b = 0; serial_port_baud_rates[b] != 0 && !device_found; b++) {
            int baud = serial_port_baud_rates[b];
            int fd = serial_port_open(ports[p].name);
            if (fd < 0) {
                log_info("Cannot open %s", ports[p].name);
                break;   /* no point trying other baud rates */
            }
            if (serial_port_configure(fd, baud) < 0) {
                serial_port_close(fd);
                continue;
            }

            log_info("Probing %s at %d baud...", ports[p].name, baud);

            for (int attempt = 0; attempt < MAX_ATTEMPTS && !device_found; attempt++) {
                for (int k = 0; k < 3; k++) {
                    serial_send_then_recv_str(fd, "LOG DISABLE\r\n", NULL, recv_buf, sizeof(recv_buf), 200);
                    safe_sleep(50 * 1000);
                }
                int len = serial_send_then_recv_str(fd, "LOG VERSION\r\n", "OK", recv_buf,
                                                    sizeof(recv_buf), CMD_REPLY_TIMEOUT_MS);
                if (len > 0) {
                    device_found = true;
                    snprintf(found_port, sizeof(found_port), "%s", ports[p].name);
                    found_baud = baud;
                    snprintf(device_info, sizeof(device_info), "%s", recv_buf);
                    serial_send_then_recv_str(fd, "LOG ENABLE\r\n", NULL, recv_buf, sizeof(recv_buf), 10);
                } else if (attempt < MAX_ATTEMPTS - 1) {
                    safe_sleep(300 * 1000);
                }
            }
            serial_port_close(fd);
        }
    }
    free_port_list(ports);

    if (!device_found) {
        log_error("No device answered on any port");
        return -1;
    }

    printf("\nDevice found\n");
    printf("  port: %s\n", found_port);
    printf("  baud: %d\n", found_baud);
    printf("%s\n", device_info);
    printf("Next:\n");
    printf("  hihost -p %s -b %d read\n", found_port, found_baud);
    printf("  hihost -p %s -b %d write \"LOG VERSION\"\n", found_port, found_baud);

    if (save) {
        if (hihost_config_save(HIHOST_INI_FILE, found_port, found_baud) != 0) {
            log_error("Cannot write ./%s", HIHOST_INI_FILE);
            return -1;
        }
        printf("Saved to ./%s; -p/-b can now be omitted.\n", HIHOST_INI_FILE);
    } else {
        printf("Run `hihost probe --save` to store these in ./%s.\n", HIHOST_INI_FILE);
    }
    return 0;
}
