#include <stdio.h>

#include "global_options.h"
#include "log.h"
#include "serial_port.h"

int cmd_list(GlobalOptions *opts, int argc, char *argv[])
{
    (void)opts; (void)argc; (void)argv;
    PortInfo *ports = NULL;
    int count = list_serial_ports(&ports);

    if (count < 0) {
        log_error("Failed to list serial ports");
        return -1;
    }
    if (count == 0) {
        printf("No serial ports found.\n");
    } else {
        print_port_list(ports, count);
    }
    free_port_list(ports);
    return 0;
}
