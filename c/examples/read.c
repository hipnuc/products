/* Change these settings, then build and run this example. */
#include <signal.h>
#include <stdio.h>
#include <inttypes.h>
#include "hipnuc_serial.h"

#ifdef _WIN32
static const char *PORT = "COM3";
#else
static const char *PORT = "/dev/ttyUSB0";
#endif
static const int BAUDRATE = 115200;
static const int TIMEOUT_MS = 2000;
static volatile sig_atomic_t stopped;
static void stop(int signal_number) { (void)signal_number; stopped = 1; }

static void print_sample(const hipnuc_sample_t *sample)
{
    printf("sample fields=0x%" PRIx64, sample->valid);
    if (sample->valid & HIPNUC_VALID_ACC)
        printf(" acc(m/s^2)=%.3f %.3f %.3f", sample->acc[0], sample->acc[1], sample->acc[2]);
    if (sample->valid & HIPNUC_VALID_GYR)
        printf(" gyr(rad/s)=%.3f %.3f %.3f", sample->gyr[0], sample->gyr[1], sample->gyr[2]);
    if (sample->valid & HIPNUC_VALID_ROLL_PITCH)
        printf(" roll/pitch(rad)=%.3f %.3f", sample->roll, sample->pitch);
    if (sample->valid & HIPNUC_VALID_YAW) printf(" yaw(rad)=%.3f", sample->yaw);
    if (sample->valid & HIPNUC_VALID_HEADING) printf(" heading(rad)=%.3f", sample->heading);
    if (sample->valid & HIPNUC_VALID_QUAT)
        printf(" quat(wxyz)=%.4f %.4f %.4f %.4f",
               sample->quat[0], sample->quat[1], sample->quat[2], sample->quat[3]);
    if (sample->valid & HIPNUC_VALID_POSITION)
        printf(" lon/lat(deg)=%.7f %.7f", sample->longitude, sample->latitude);
    if (sample->valid & HIPNUC_VALID_TEMPERATURE) printf(" temp(C)=%.2f", sample->temperature);
    putchar('\n');
}

int main(void)
{
    hipnuc_serial_t device = {0};
    hipnuc_sample_t sample;
    int result = 0, waiting = 0;
    signal(SIGINT, stop);
    if (hipnuc_serial_open(&device, PORT, BAUDRATE) < 0) {
        fprintf(stderr, "%s: %s\n", PORT, hipnuc_serial_last_error(&device));
        return 1;
    }
    fprintf(stderr, "Connected to %s at %d baud. Ctrl-C stops.\n", PORT, BAUDRATE);
    while (!stopped) {
        int received = hipnuc_serial_read_sample(&device, &sample, TIMEOUT_MS);
        if (received < 0) {
            fprintf(stderr, "%s\n", hipnuc_serial_last_error(&device));
            result = 1;
            break;
        }
        if (received == 0) {
            if (!waiting && !stopped)
                fputs("No valid sample; check baudrate, device output and TIMEOUT_MS. Waiting...\n", stderr);
            waiting = 1;
            continue;
        }
        if (waiting) fputs("Data resumed.\n", stderr);
        waiting = 0;
        print_sample(&sample);
        /* Process other valid fields here. Each sample describes one packet. */
        if (fflush(stdout) != 0) { perror("Output failed"); result = 1; break; }
    }
    hipnuc_serial_close(&device);
    return result ? result : stopped ? 130 : 0;
}
