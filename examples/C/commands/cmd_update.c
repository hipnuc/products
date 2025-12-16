#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "global_options.h"
#include "serial_port.h"
#include "fw_downloader/hex2bin.h"
#include "fw_downloader/kboot.h"
#include "log.h"
#include "cmd_utils.h"

#define MAX_PING_ATTEMPTS 10
#define BAR_WIDTH 50
#define CMD_REPLAY_TIMEOUT_MS (100)

int cmd_update(GlobalOptions *opts, int argc, char *argv[]) {
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