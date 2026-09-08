// C++ calls the same C API; no additional device class is required.
#include <csignal>
#include <iomanip>
#include <iostream>
#include "hipnuc_serial.h"

#ifdef _WIN32
static const char *PORT = "COM3";
#else
static const char *PORT = "/dev/ttyUSB0";
#endif
static const int BAUDRATE = 115200;
static const int TIMEOUT_MS = 2000;
static volatile std::sig_atomic_t stopped;
static void stop(int) { stopped = 1; }

static void print_sample(const hipnuc_sample_t &sample)
{
    std::cout << "sample fields=0x" << std::hex << sample.valid << std::dec << std::setprecision(10);
    if (sample.valid & HIPNUC_VALID_ACC)
        std::cout << " acc(m/s^2)=" << sample.acc[0] << ' ' << sample.acc[1] << ' ' << sample.acc[2];
    if (sample.valid & HIPNUC_VALID_GYR)
        std::cout << " gyr(rad/s)=" << sample.gyr[0] << ' ' << sample.gyr[1] << ' ' << sample.gyr[2];
    if (sample.valid & HIPNUC_VALID_ROLL_PITCH)
        std::cout << " roll/pitch(rad)=" << sample.roll << ' ' << sample.pitch;
    if (sample.valid & HIPNUC_VALID_YAW) std::cout << " yaw(rad)=" << sample.yaw;
    if (sample.valid & HIPNUC_VALID_HEADING) std::cout << " heading(rad)=" << sample.heading;
    if (sample.valid & HIPNUC_VALID_QUAT)
        std::cout << " quat(wxyz)=" << sample.quat[0] << ' ' << sample.quat[1] << ' '
                  << sample.quat[2] << ' ' << sample.quat[3];
    if (sample.valid & HIPNUC_VALID_POSITION)
        std::cout << " lon/lat(deg)=" << sample.longitude << ' ' << sample.latitude;
    if (sample.valid & HIPNUC_VALID_TEMPERATURE) std::cout << " temp(C)=" << sample.temperature;
    std::cout << '\n' << std::flush;
}

int main()
{
    hipnuc_serial_t device = {};
    hipnuc_sample_t sample;
    int result = 0;
    bool waiting = false;
    std::signal(SIGINT, stop);
    if (hipnuc_serial_open(&device, PORT, BAUDRATE) < 0) {
        std::cerr << PORT << ": " << hipnuc_serial_last_error(&device) << '\n';
        return 1;
    }
    std::cerr << "Connected to " << PORT << " at " << BAUDRATE << " baud. Ctrl-C stops.\n";
    while (!stopped) {
        int received = hipnuc_serial_read_sample(&device, &sample, TIMEOUT_MS);
        if (received < 0) {
            std::cerr << hipnuc_serial_last_error(&device) << '\n';
            result = 1;
            break;
        }
        if (received == 0) {
            if (!waiting && !stopped)
                std::cerr << "No valid sample; check baudrate, device output and TIMEOUT_MS. Waiting...\n";
            waiting = true;
            continue;
        }
        if (waiting) std::cerr << "Data resumed.\n";
        waiting = false;
        print_sample(sample);
        // Process other valid fields here. Each sample describes one packet.
        if (!std::cout) { std::cerr << "Output failed.\n"; result = 1; break; }
    }
    hipnuc_serial_close(&device);
    return result ? result : stopped ? 130 : 0;
}
