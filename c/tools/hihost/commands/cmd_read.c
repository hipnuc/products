/* read: decode the binary and NMEA stream, display it and optionally record
 * raw bytes (-r) and JSON lines (-j). Ctrl+C stops. */
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "global_options.h"
#include "hihost_config.h"
#include "hipnuc_dec.h"
#include "hipnuc_json.h"
#include "hipnuc_sample.h"
#include "log.h"
#include "nmea_dec.h"
#include "serial_port.h"

#define DISPLAY_INTERVAL_S 0.05
#define JSON_BUF_SIZE 1024

static volatile sig_atomic_t running = 1;

static void on_signal(int sig)
{
    (void)sig;
    running = 0;
}

/* Format one sample; returns 0 and fills buf, or -1 when it does not fit. */
static int sample_to_json(const hipnuc_sample_t *s, char *buf, size_t size)
{
    int required = hipnuc_json_sample(s, NULL, 0);
    if (required < 0 || (size_t)required + 1 > size) {
        return -1;
    }
    return hipnuc_json_sample(s, buf, size) >= 0 ? 0 : -1;
}

static double seconds_between(const struct timespec *a, const struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) + (double)(b->tv_nsec - a->tv_nsec) / 1e9;
}

int cmd_read(GlobalOptions *opts, int argc, char *argv[])
{
    (void)argc; (void)argv;
    uint8_t recv_buf[1024];
    char json_binary[JSON_BUF_SIZE] = {0};
    char json_nmea[JSON_BUF_SIZE] = {0};
    hipnuc_raw_t hipnuc_raw;
    nmea_raw_t nmea_raw;
    hipnuc_sample_t sample;
    struct timespec rate_time, display_time, now;
    long frame_count = 0;
    int frame_rate = 0;
    int result = 0;
    FILE *raw_fp = NULL;
    FILE *json_fp = NULL;
    struct sigaction sa;

    memset(&hipnuc_raw, 0, sizeof(hipnuc_raw));
    memset(&nmea_raw, 0, sizeof(nmea_raw));

    int fd = serial_port_open(opts->port_name);
    if (fd < 0 || serial_port_configure(fd, opts->baud_rate) < 0) {
        log_error("Cannot open %s at %d baud", opts->port_name, opts->baud_rate);
        if (fd >= 0) serial_port_close(fd);
        return -1;
    }

    if (opts->record_raw_file) {
        raw_fp = fopen(opts->record_raw_file, "wb");
        if (!raw_fp) {
            log_error("Cannot open %s for writing", opts->record_raw_file);
            serial_port_close(fd);
            return -1;
        }
        printf("Recording raw bytes to %s\n", opts->record_raw_file);
    }
    if (opts->record_json_file) {
        json_fp = fopen(opts->record_json_file, "w");
        if (!json_fp) {
            log_error("Cannot open %s for writing", opts->record_json_file);
            if (raw_fp) fclose(raw_fp);
            serial_port_close(fd);
            return -1;
        }
        printf("Recording JSON lines to %s\n", opts->record_json_file);
    }

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    printf("Reading %s at %d baud. Press Ctrl+C to stop.\n", opts->port_name, opts->baud_rate);

    /* Make sure the device is streaming (a previous `write` may have disabled it). */
    {
        char reply[256];
        serial_send_then_recv_str(fd, "LOG ENABLE\r\n", NULL, reply, sizeof(reply), 100);
    }

    clock_gettime(CLOCK_MONOTONIC, &rate_time);
    display_time = rate_time;

    while (running) {
        bool new_binary = false;
        bool new_nmea = false;

        int len = serial_port_read(fd, recv_buf, sizeof(recv_buf));
        if (len < 0) {
            log_error("Serial read failed");
            result = -1;
            break;
        }
        if (len > 0 && raw_fp) {
            fwrite(recv_buf, 1, (size_t)len, raw_fp);
        }

        for (int i = 0; i < len; i++) {
            if (hipnuc_input(&hipnuc_raw, recv_buf[i]) > 0 &&
                hipnuc_sample_from_raw(&hipnuc_raw, &sample)) {
                frame_count++;
                if (sample_to_json(&sample, json_binary, sizeof(json_binary)) == 0) {
                    new_binary = true;
                    if (json_fp && fprintf(json_fp, "%s\n", json_binary) < 0) {
                        log_error("Cannot write %s", opts->record_json_file);
                        result = -1;
                        running = 0;
                        break;
                    }
                }
            }
            if (nmea_input(&nmea_raw, recv_buf[i]) > 0 &&
                hipnuc_sample_from_nmea(&nmea_raw, &sample)) {
                frame_count++;
                if (sample_to_json(&sample, json_nmea, sizeof(json_nmea)) == 0) {
                    new_nmea = true;
                    if (json_fp && fprintf(json_fp, "%s\n", json_nmea) < 0) {
                        log_error("Cannot write %s", opts->record_json_file);
                        result = -1;
                        running = 0;
                        break;
                    }
                }
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        if ((new_binary || new_nmea) && seconds_between(&display_time, &now) >= DISPLAY_INTERVAL_S) {
            printf("\033[H\033[J");   /* clear screen */
            if (json_binary[0]) printf("%s\n", json_binary);
            if (json_nmea[0]) printf("%s\n", json_nmea);
            printf("frames/s: %d   crc errors: %u\n", frame_rate, (unsigned)hipnuc_raw.crc_error_count);
            fflush(stdout);
            display_time = now;
        }

        double elapsed = seconds_between(&rate_time, &now);
        if (elapsed >= 1.0) {
            frame_rate = (int)((double)frame_count / elapsed);
            frame_count = 0;
            rate_time = now;
            if (raw_fp) fflush(raw_fp);
            if (json_fp) fflush(json_fp);
        }

        if (len == 0) {
            safe_sleep(1000);
        }
    }

    printf("\n");
    if (raw_fp) fclose(raw_fp);
    if (json_fp) fclose(json_fp);
    serial_port_close(fd);
    return result;
}
