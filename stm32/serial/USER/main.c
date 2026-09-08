/* STM32F103 / StdPeriph. Device TX -> PA3, RX <- PA2, GND -> GND.
 * Console: USART1 PA9, 115200 8N1. Edit settings and the application block. */
#include <stdio.h>
#include "hipnuc_board.h"

#define IMU_BAUDRATE       115200U
#define PRINT_PERIOD_MS   200U       /* sample display; 0 disables it */
#define REPORT_PERIOD_MS  2000U      /* receive health, including no data */

/* Display only fields carried by this sample, without collecting old values. */
static void print_sample(const hipnuc_sample_t *sample)
{
    if (sample->valid & HIPNUC_VALID_ROLL_PITCH) {
        printf("roll %.1f pitch %.1f deg", sample->roll * 57.29578f,
               sample->pitch * 57.29578f);
        if (sample->valid & HIPNUC_VALID_YAW)
            printf(" yaw %.1f deg", sample->yaw * 57.29578f);
    } else if (sample->valid & HIPNUC_VALID_ACC) {
        printf("acc %.2f %.2f %.2f m/s2", sample->acc[0], sample->acc[1], sample->acc[2]);
    } else if (sample->valid & HIPNUC_VALID_GYR) {
        printf("gyr %.2f %.2f %.2f rad/s", sample->gyr[0], sample->gyr[1], sample->gyr[2]);
    } else {
        printf("sample source %u", (unsigned)sample->source);
    }
    printf("\r\n");
}

int main(void)
{
    hipnuc_sample_t sample;
    hipnuc_board_stats_t previous = {0};
    uint32_t last_print = 0, last_report = 0;
    int was_receiving = -1;

    hipnuc_board_init(IMU_BAUDRATE);
    printf("HiPNUC serial: %u baud, waiting for data\r\n", (unsigned)IMU_BAUDRATE);
    while (1) {
        uint32_t now;
        if (hipnuc_board_poll(&sample)) {
            /* ---- your application: this is one NEW sample ---------------
             * Check HIPNUC_VALID_* before reading a field. For example:
             * if (sample.valid & HIPNUC_VALID_ACC) use sample.acc[0] (m/s2).
             * Euler angles are rad; angular velocity is rad/s.
             * ------------------------------------------------------------- */
            now = hipnuc_board_millis();
            if (PRINT_PERIOD_MS && now - last_print >= PRINT_PERIOD_MS) {
                last_print = now;
                print_sample(&sample);
            }
        }

        now = hipnuc_board_millis();
        if (now - last_report >= REPORT_PERIOD_MS) {
            hipnuc_board_stats_t current = *hipnuc_board_stats();
            uint32_t bytes = current.bytes - previous.bytes;
            uint32_t frames = current.frames - previous.frames;
            if (!bytes) {
                printf("no bytes: check TX->PA3, GND, baudrate and device output\r\n");
            } else if (!frames) {
                printf("bytes received, no valid frames: check baudrate and output format\r\n");
            } else {
                printf("%s: %lu samples/s\r\n", was_receiving == 0 ? "recovered" : "receiving",
                       (unsigned long)(frames * 1000U / (now - last_report)));
            }
            if (current.overruns != previous.overruns)
                printf("RX overrun: poll faster or reduce printing/output rate\r\n");
            if (current.hardware_errors != previous.hardware_errors)
                printf("UART errors: %lu; check baudrate, wiring and interrupt load\r\n",
                       (unsigned long)(current.hardware_errors - previous.hardware_errors));
            if (current.crc_errors != previous.crc_errors || current.invalid_frames != previous.invalid_frames)
                printf("bad frames: CRC %lu, format %lu in this window\r\n",
                       (unsigned long)(current.crc_errors - previous.crc_errors),
                       (unsigned long)(current.invalid_frames - previous.invalid_frames));
            was_receiving = frames != 0;
            previous = current;
            last_report = now;
        }
    }
}
