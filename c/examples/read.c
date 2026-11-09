/* Reads a HiPNUC device on a serial port and prints what it sends.
 * Change PORT and BAUDRATE, then build and run. Ctrl-C stops. */
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include "hipnuc_serial.h"

#ifdef _WIN32
static const char *PORT = "COM3";
#else
static const char *PORT = "/dev/ttyUSB0";
#endif
static const int BAUDRATE = 115200;   /* the factory setting */
static const int TIMEOUT_MS = 2000;

static volatile sig_atomic_t stopped;
static void stop(int signal_number) { (void)signal_number; stopped = 1; }

static void print_data(const hipnuc_data_t *imu)
{
    printf("fields=0x%" PRIx64, imu->valid);
    if (imu->valid & HIPNUC_VALID_ACC) printf(" acc(m/s^2)=%.3f %.3f %.3f", imu->acc[0], imu->acc[1], imu->acc[2]);
    if (imu->valid & HIPNUC_VALID_GYR) printf(" gyr(rad/s)=%.3f %.3f %.3f", imu->gyr[0], imu->gyr[1], imu->gyr[2]);
    if (imu->valid & HIPNUC_VALID_ROLL_PITCH)
        printf(" roll/pitch(deg)=%.2f %.2f", imu->roll * HIPNUC_RAD2DEG, imu->pitch * HIPNUC_RAD2DEG);
    if (imu->valid & HIPNUC_VALID_YAW) printf(" yaw(deg)=%.2f", imu->yaw * HIPNUC_RAD2DEG);
    if (imu->valid & HIPNUC_VALID_HEADING) printf(" heading(deg)=%.2f", imu->heading * HIPNUC_RAD2DEG);
    if (imu->valid & HIPNUC_VALID_QUAT)
        printf(" quat(wxyz)=%.4f %.4f %.4f %.4f", imu->quat[0], imu->quat[1], imu->quat[2], imu->quat[3]);
    if (imu->valid & HIPNUC_VALID_POSITION) printf(" lon/lat(deg)=%.7f %.7f", imu->longitude, imu->latitude);
    if (imu->valid & HIPNUC_VALID_TEMPERATURE) printf(" temp(C)=%.1f", imu->temperature);
    putchar('\n');
}

/* A read-only query: who is sending (nothing is changed). */
static void print_identity(hipnuc_serial_t *dev)
{
    const char *name, *app;
    if (hipnuc_serial_command(dev, "LOG VERSION", 0) != 0) {
        fprintf(stderr, "LOG VERSION: %s\n", hipnuc_serial_last_error(dev));
        return;
    }
    name = hipnuc_serial_reply_value(dev, "PNAME");
    fprintf(stderr, "Device %s", name ? name : "(unknown)");
    app = hipnuc_serial_reply_value(dev, "APP_VER");   /* 173 is 1.7.3 */
    if (app) fprintf(stderr, ", firmware %s", app);
    fputc('\n', stderr);
}

int main(void)
{
    hipnuc_serial_t dev = {0};
    hipnuc_data_t imu;
    int result = 0, waiting = 0, identified = 0;

    signal(SIGINT, stop);
    if (hipnuc_serial_open(&dev, PORT, BAUDRATE) < 0) {
        fprintf(stderr, "%s: %s\n", PORT, hipnuc_serial_last_error(&dev));
        return 1;
    }
    fprintf(stderr, "Connected to %s at %d baud. Ctrl-C stops.\n", PORT, BAUDRATE);
    while (!stopped) {
        int received = hipnuc_serial_read(&dev, &imu, TIMEOUT_MS);
        if (received < 0) {
            fprintf(stderr, "%s\n", hipnuc_serial_last_error(&dev));
            result = 1;
            break;
        }
        if (received == 0) {
            if (!waiting && !stopped) {
                if (dev.bytes_read == 0)
                    fputs("No bytes: check power, the cable (device TX to adapter RX) and the port.\n", stderr);
                else if (dev.decoder.unsupported > 0)
                    fputs("Frames this SDK does not decode: check the output setting (LOG HI83 MAP).\n", stderr);
                else
                    fputs("Bytes but no valid frames: check the baud rate and that the device outputs data.\n", stderr);
                fprintf(stderr, "%" PRIu64 " bytes, %lu frames, %lu CRC errors. Waiting...\n", dev.bytes_read,
                        (unsigned long)dev.decoder.frames, (unsigned long)dev.decoder.crc_errors);
            }
            waiting = 1;
            continue;
        }
        if (waiting) fputs("Data resumed.\n", stderr);
        waiting = 0;
        print_data(&imu);
        if (fflush(stdout) != 0) {
            perror("Output failed");
            result = 1;
            break;
        }
        /* Once data flows, ask who sends it. Messages that arrive meanwhile are dropped. */
        if (!identified) {
            identified = 1;
            print_identity(&dev);
        }
    }
    hipnuc_serial_close(&dev);
    return result ? result : stopped ? 130 : 0;
}
