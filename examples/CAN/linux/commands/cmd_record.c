#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <linux/can.h>
#include "../can_interface.h"
#include "../log.h"
#include "../utils.h"
#include "../config.h"
#include "hipnuc_can_common.h"
#include "hipnuc_j1939_parser.h"
#include "canopen_parser.h"

// Lock-free ring buffer
typedef struct {
    hipnuc_can_frame_t *frames;
    size_t capacity;
    volatile size_t write_pos;
    volatile size_t read_pos;
} ring_buffer_t;

typedef struct {
    ring_buffer_t ring;
    FILE *file;
    volatile bool running;
    uint64_t total_frames;
} writer_ctx_t;

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

static bool ring_init(ring_buffer_t *r, size_t capacity)
{
    r->frames = calloc(capacity, sizeof(hipnuc_can_frame_t));
    if (!r->frames) return false;
    r->capacity = capacity;
    r->write_pos = 0;
    r->read_pos = 0;
    return true;
}

static void ring_destroy(ring_buffer_t *r)
{
    free(r->frames);
}

static size_t ring_available(ring_buffer_t *r)
{
    size_t w = r->write_pos;
    size_t rd = r->read_pos;
    return (w >= rd) ? (w - rd) : (r->capacity - rd + w);
}

static bool ring_push(ring_buffer_t *r, const hipnuc_can_frame_t *frame)
{
    size_t next_write = (r->write_pos + 1) % r->capacity;
    if (next_write == r->read_pos) {
        return false; // Full
    }
    r->frames[r->write_pos] = *frame;
    __sync_synchronize();
    r->write_pos = next_write;
    return true;
}

static size_t ring_pop_batch(ring_buffer_t *r, hipnuc_can_frame_t *out, size_t max_count)
{
    size_t avail = ring_available(r);
    if (avail == 0) return 0;
    
    size_t count = avail < max_count ? avail : max_count;
    size_t rd = r->read_pos;
    
    for (size_t i = 0; i < count; ++i) {
        out[i] = r->frames[rd];
        rd = (rd + 1) % r->capacity;
    }
    
    __sync_synchronize();
    r->read_pos = rd;
    return count;
}

static void *writer_thread(void *arg)
{
    writer_ctx_t *ctx = (writer_ctx_t *)arg;
    hipnuc_can_frame_t batch[256];
    char json_buf[256 * 512]; // Pre-allocated buffer for batch JSON
    
    while (ctx->running || ring_available(&ctx->ring) > 0) {
        size_t n = ring_pop_batch(&ctx->ring, batch, 256);
        if (n == 0) {
            usleep(1000); // 1ms
            continue;
        }
        
        char *ptr = json_buf;
        size_t remaining = sizeof(json_buf);
        
        for (size_t i = 0; i < n; ++i) {
            can_sensor_data_t data;
            memset(&data, 0, sizeof(data));
            data.node_id = hipnuc_can_extract_node_id(batch[i].can_id);
            data.hw_ts_us = batch[i].hw_ts_us;
            
            int msg_type = (batch[i].can_id & HIPNUC_CAN_EFF_FLAG)
                         ? hipnuc_j1939_parse_frame(&batch[i], &data)
                         : canopen_parse_frame(&batch[i], &data);
            
            if (msg_type == CAN_MSG_UNKNOWN || msg_type == CAN_MSG_ERROR) {
                continue;
            }
            
            can_json_output_t json;
            if (hipnuc_can_to_json(&data, msg_type, &json) <= 0) {
                continue;
            }
            
            if (json.length >= remaining) {
                // Flush current buffer
                if (ptr > json_buf) {
                    fwrite(json_buf, 1, ptr - json_buf, ctx->file);
                }
                ptr = json_buf;
                remaining = sizeof(json_buf);
            }
            
            memcpy(ptr, json.buffer, json.length);
            ptr += json.length;
            remaining -= json.length;
        }
        
        // Flush remaining
        if (ptr > json_buf) {
            fwrite(json_buf, 1, ptr - json_buf, ctx->file);
        }
        
        ctx->total_frames += n;
    }
    
    fflush(ctx->file);
    return NULL;
}

int cmd_record(int argc, char *argv[])
{
    const char *out_path = NULL;
    size_t buf_frames = 100;
    
    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--out") == 0) && i + 1 < argc) {
            out_path = argv[++i];
        } else if (strcmp(argv[i], "--buf-frames") == 0 && i + 1 < argc) {
            buf_frames = (size_t)strtoull(argv[++i], NULL, 10);
        }
    }
    
    if (!out_path) {
        log_error("Missing output file (-o FILE)");
        return -1;
    }

    const char *ifname = config_get_interface();
    int sockfd = can_open_socket(ifname);
    if (sockfd < 0) {
        utils_print_can_setup_hint(ifname);
        return -1;
    }

    FILE *file = fopen(out_path, "w");
    if (!file) {
        log_error("Failed to open %s: %s", out_path, strerror(errno));
        can_close_socket(sockfd);
        return -1;
    }
    
    // Set large buffer for file I/O
    setvbuf(file, NULL, _IOFBF, 1024 * 1024);

    writer_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    if (!ring_init(&ctx.ring, buf_frames)) {
        log_error("Failed to allocate ring buffer");
        fclose(file);
        can_close_socket(sockfd);
        return -1;
    }
    ctx.file = file;
    ctx.running = true;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    pthread_t writer;
    pthread_create(&writer, NULL, writer_thread, &ctx);

    log_info("Recording JSON on %s -> %s", ifname, out_path);

    struct can_frame frames[256];
    uint64_t ts_us[256];
    size_t batch_cap = sizeof(frames) / sizeof(frames[0]);
    
    uint64_t rx_frames = 0;
    uint64_t dropped_frames = 0;
    uint64_t last_print_rx = 0;
    uint32_t last_print_ms = utils_get_timestamp_ms();

    while (g_running) {
        int r = can_receive_frames_ts(sockfd, frames, ts_us, batch_cap, 100);
        
        if (r < 0) {
            log_error("Receive error");
            break;
        }
        
        if (r == 0) {
            uint32_t now_ms = utils_get_timestamp_ms();
            if (now_ms - last_print_ms >= 1000) {
                uint64_t delta_rx = rx_frames - last_print_rx;
                double fps = (double)delta_rx * 1000.0 / (double)(now_ms - last_print_ms);
                size_t queue_used = ring_available(&ctx.ring);
                printf("\rrx=%llu written=%llu dropped=%llu fps=%.1f queue=%zu/%zu   ",
                       (unsigned long long)rx_frames,
                       (unsigned long long)ctx.total_frames,
                       (unsigned long long)dropped_frames,
                       fps, queue_used, buf_frames);
                fflush(stdout);
                last_print_rx = rx_frames;
                last_print_ms = now_ms;
            }
            continue;
        }

        for (int i = 0; i < r; ++i) {
            hipnuc_can_frame_t hipnuc_frame;
            utils_linux_can_to_hipnuc_can(&frames[i], ts_us[i], &hipnuc_frame);
            
            if (!ring_push(&ctx.ring, &hipnuc_frame)) {
                dropped_frames++;
            }
        }
        rx_frames += r;

        uint32_t now_ms = utils_get_timestamp_ms();
        if (now_ms - last_print_ms >= 1000) {
            uint64_t delta_rx = rx_frames - last_print_rx;
            double fps = (double)delta_rx * 1000.0 / (double)(now_ms - last_print_ms);
            size_t queue_used = ring_available(&ctx.ring);
            printf("\rrx=%llu written=%llu dropped=%llu fps=%.1f queue=%zu/%zu   ",
                   (unsigned long long)rx_frames,
                   (unsigned long long)ctx.total_frames,
                   (unsigned long long)dropped_frames,
                   fps, queue_used, buf_frames);
            fflush(stdout);
            last_print_rx = rx_frames;
            last_print_ms = now_ms;
        }
    }

    printf("\n");
    log_info("Stopping writer thread...");
    ctx.running = false;
    pthread_join(writer, NULL);

    log_info("Recorded %llu frames (%llu dropped)", 
             (unsigned long long)ctx.total_frames,
             (unsigned long long)dropped_frames);

    ring_destroy(&ctx.ring);
    fclose(file);
    can_close_socket(sockfd);
    return 0;
}
