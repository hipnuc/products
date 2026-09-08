// C++ calls the same C API; no additional device class is required.
#include <csignal>
#include <iostream>
#include "hipnuc_serial.h"

#ifdef _WIN32
static const char *PORT = "COM3";
#else
static const char *PORT = "/dev/ttyUSB0";
#endif
static const int BAUDRATE = 115200;
static volatile std::sig_atomic_t stopped;
static void stop(int) { stopped = 1; }

int main()
{
    hipnuc_serial_t device = {};
    hipnuc_sample_t sample;
    int result = 0;
    std::signal(SIGINT, stop);
    if (hipnuc_serial_open(&device, PORT, BAUDRATE) < 0) {
        std::cerr << PORT << ": " << hipnuc_serial_last_error(&device) << '\n';
        return 1;
    }
    while (!stopped) {
        int received = hipnuc_serial_read_sample(&device, &sample, 200);
        if (received < 0) {
            std::cerr << hipnuc_serial_last_error(&device) << '\n';
            result = 1;
            break;
        }
        if (received == 0) continue;
        if (sample.valid & HIPNUC_VALID_ACC)
            std::cout << "Acceleration (m/s^2): " << sample.acc[0] << ' '
                      << sample.acc[1] << ' ' << sample.acc[2] << '\n';
        // Use the other valid fields here. Each sample is new, not a cache.
    }
    hipnuc_serial_close(&device);
    return stopped ? 130 : result;
}
