/* Change these two settings, then build and run this example. */
#include <signal.h>
#include <stdio.h>
#include "hipnuc_serial.h"

#ifdef _WIN32
static const char *PORT = "COM3";
#else
static const char *PORT = "/dev/ttyUSB0";
#endif
static const int BAUDRATE = 115200;
static volatile sig_atomic_t stopped;
static void stop(int signal_number) { (void)signal_number; stopped = 1; }

int main(void)
{
    hipnuc_serial_t device = {0};
    hipnuc_sample_t sample;
    int result = 0;
    signal(SIGINT, stop);
    if (hipnuc_serial_open(&device, PORT, BAUDRATE) < 0) {
        fprintf(stderr, "%s: %s\n", PORT, hipnuc_serial_last_error(&device));
        return 1;
    }
    while (!stopped) {
        int received = hipnuc_serial_read_sample(&device, &sample, 200);
        if (received < 0) {
            fprintf(stderr, "%s\n", hipnuc_serial_last_error(&device));
            result = 1;
            break;
        }
        if (received == 0) continue;
        if (sample.valid & HIPNUC_VALID_ACC)
            printf("Acceleration (m/s^2): %.3f %.3f %.3f\n",
                   sample.acc[0], sample.acc[1], sample.acc[2]);
        /* Use the other valid fields here. Each sample is new, not a cache. */
    }
    hipnuc_serial_close(&device);
    return stopped ? 130 : result;
}
