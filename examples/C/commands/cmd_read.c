#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include "global_options.h"
#include "serial_port.h"
#include "hipnuc_dec.h"
#include "nmea_dec.h"
#include "log.h"
#include "cmd_utils.h"

#define DISPLAY_UPDATE_INTERVAL 0.05

int cmd_read(GlobalOptions *opts, int argc, char *argv[]) {
    (void)argc; (void)argv;
    int fd = -1;
    uint8_t recv_buf[1024];
    char log_buf[512];
    hipnuc_raw_t hipnuc_raw = {0};
    nmea_raw_t nmea_raw = {0};
    struct timespec last_time, current_time, last_display_time;
    long long frame_count = 0;
    int frame_rate = 0;
    double elapsed_time = 0.0;
    
    // Recording file pointers
    FILE *raw_fp = NULL;
    FILE *json_fp = NULL;

    if ((fd = serial_port_open(opts->port_name)) < 0 || serial_port_configure(fd, opts->baud_rate) < 0) {
        log_error("Failed to open or configure port %s with %d", opts->port_name, opts->baud_rate);
        return -1;
    }

    // Open recording files and show paths immediately
    if (opts->record_raw_file) {
        raw_fp = fopen(opts->record_raw_file, "wb");
        if (!raw_fp) {
            log_error("Failed to open raw data recording file: %s", opts->record_raw_file);
            serial_port_close(fd);
            return -1;
        }
        printf("Raw data will be recorded to: %s\n", opts->record_raw_file);
    }
    
    if (opts->record_json_file) {
        json_fp = fopen(opts->record_json_file, "w");
        if (!json_fp) {
            log_error("Failed to open JSON data recording file: %s", opts->record_json_file);
            if (raw_fp) fclose(raw_fp);
            serial_port_close(fd);
            return -1;
        }
        printf("JSON data will be recorded to: %s\n", opts->record_json_file);
    }

    printf("Reading from port %s at %d baud. Press CTRL+C to exit.\n", opts->port_name, opts->baud_rate);
    
    // Countdown before starting
    printf("Starting in: ");
    fflush(stdout);
    for (int i = 3; i >= 1; i--) {
        printf("%d", i);
        fflush(stdout);
        sleep(1);
        if (i > 1) {
            printf(", ");
            fflush(stdout);
        }
    }
    printf("\nStarting data recording and display...\n\n");

    serial_send_then_recv_str(fd, "LOG ENABLE\r\n", "OK\r\n", recv_buf, sizeof(recv_buf), 200);

    clock_gettime(CLOCK_MONOTONIC, &last_time);
    last_display_time = last_time;

    // Main reading loop
    while (1) {
        bool new_hipnuc_data = false;
        bool new_nmea_data = false;

        // Read data from serial port
        int len = serial_port_read(fd, (char *)recv_buf, sizeof(recv_buf));
        if (len > 0) {
            // Record raw data if enabled
            if (raw_fp) {
                fwrite(recv_buf, 1, len, raw_fp);
                fflush(raw_fp);  // Ensure data is written immediately
            }
            
            for (int i = 0; i < len; i++) {
                // Process HipNuc data
                if (hipnuc_input(&hipnuc_raw, recv_buf[i]) > 0) {
                    new_hipnuc_data = true;
                    frame_count++;
                    
                    // Record JSON data immediately if enabled
                    if (json_fp) {
                        hipnuc_dump_packet(&hipnuc_raw, log_buf, sizeof(log_buf));
                        fprintf(json_fp, "%s\n", log_buf);
                        fflush(json_fp);  // Ensure data is written immediately
                    }
                }
                
                // Process NMEA data
                if (input_nmea(&nmea_raw, recv_buf[i]) > 0) {
                    new_nmea_data = true;
                    frame_count++;
                    
                    // Record JSON data immediately if enabled
                    if (json_fp) {
                        nmea_dec_dump_msg(&nmea_raw, log_buf, sizeof(log_buf));
                        fprintf(json_fp, "%s\n", log_buf);
                        fflush(json_fp);  // Ensure data is written immediately
                    }
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

    // Close recording files
    if (raw_fp) {
        fclose(raw_fp);
    }
    
    if (json_fp) {
        fclose(json_fp);
    }

    serial_port_close(fd);
    return 0;
}