// The same as read.c from C++: the C API needs no wrapper class.
#include <csignal>
#include <cstdio>
#include <iostream>
#include "hipnuc_serial.h"

#ifdef _WIN32
static const char *PORT = "COM3";
#else
static const char *PORT = "/dev/ttyUSB0";
#endif
static const int BAUDRATE = 115200;   // the factory setting
static const int TIMEOUT_MS = 2000;

static volatile std::sig_atomic_t stopped;
static void stop(int) { stopped = 1; }

static void print_data(const hipnuc_data_t &imu)
{
    std::cout << "fields=0x" << std::hex << imu.valid << std::dec;
    if (imu.valid & HIPNUC_VALID_ACC) std::cout << " acc(m/s^2)=" << imu.acc[0] << ' ' << imu.acc[1] << ' ' << imu.acc[2];
    if (imu.valid & HIPNUC_VALID_GYR) std::cout << " gyr(rad/s)=" << imu.gyr[0] << ' ' << imu.gyr[1] << ' ' << imu.gyr[2];
    if (imu.valid & HIPNUC_VALID_ROLL_PITCH)
        std::cout << " roll/pitch(deg)=" << imu.roll * HIPNUC_RAD2DEG << ' ' << imu.pitch * HIPNUC_RAD2DEG;
    if (imu.valid & HIPNUC_VALID_YAW) std::cout << " yaw(deg)=" << imu.yaw * HIPNUC_RAD2DEG;
    if (imu.valid & HIPNUC_VALID_QUAT)
        std::cout << " quat(wxyz)=" << imu.quat[0] << ' ' << imu.quat[1] << ' ' << imu.quat[2] << ' ' << imu.quat[3];
    std::cout << '\n' << std::flush;
}

int main()
{
    hipnuc_serial_t dev = {};
    hipnuc_data_t imu;
    bool identified = false;
    std::signal(SIGINT, stop);
    if (hipnuc_serial_open(&dev, PORT, BAUDRATE) < 0) {
        std::cerr << PORT << ": " << hipnuc_serial_last_error(&dev) << '\n';
        return 1;
    }
    while (!stopped) {
        const int received = hipnuc_serial_read(&dev, &imu, TIMEOUT_MS);
        if (received < 0) {
            std::cerr << hipnuc_serial_last_error(&dev) << '\n';
            break;
        }
        if (received == 0) {
            std::cerr << "No data in " << TIMEOUT_MS << " ms (" << dev.bytes_read << " bytes, "
                      << dev.decoder.frames << " frames)\n";
            continue;
        }
        print_data(imu);
        if (!identified) {
            identified = true;
            if (hipnuc_serial_command(&dev, "LOG VERSION", 0) == 0) {
                const char *name = hipnuc_serial_reply_value(&dev, "PNAME");
                std::cerr << "Device " << (name ? name : "(unknown)") << '\n';
            }
        }
    }
    hipnuc_serial_close(&dev);
    return stopped ? 130 : 1;
}
