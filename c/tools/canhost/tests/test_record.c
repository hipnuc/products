#include "cli.h"
#include "hipnuc_sample.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
volatile sig_atomic_t canhost_stop;

int canhost_write_sample(FILE *, const hipnuc_sample_t *, uint64_t, char **, size_t *);

int main(void)
{
    hipnuc_sample_t sample = {0};
    sample.source = HIPNUC_SOURCE_HI83;
    sample.valid = (UINT64_C(1) << 39) - 1;
    sample.utc.year = 2026;
    sample.utc.month = sample.utc.day = 1;
    sample.node_id = 255;
    sample.longitude = sample.gnss_longitude = 123.1234567;
    sample.latitude = sample.gnss_latitude = 32.1234567;
    sample.altitude_msl = sample.gnss_altitude_msl = 120.123456;
    sample.quat[0] = 1;
    sample.roll = sample.pitch = sample.yaw = sample.heading = 1.2345678f;
    sample.temperature = 23.45678f;
    sample.pressure = 101234.56f;
    sample.device_time_us = UINT64_MAX;
    sample.gps_tow_ms = UINT32_MAX;
    for (int i = 0; i < 3; ++i) {
        sample.acc[i] = sample.gyr[i] = sample.mag[i] = 12.345678f;
        sample.vel_enu[i] = sample.acc_enu[i] = sample.gnss_vel_enu[i] = 23.456789f;
        sample.heave_m[i] = sample.heave_hz[i] = 1.2345678f;
    }
    sample.nmea_status = 'A';
    sample.nmea_mode = 'A';
    FILE *file = tmpfile();
    assert(file);
    char *json = NULL;
    size_t capacity = 0;
    assert(canhost_write_sample(file, &sample, 123, &json, &capacity) == 0);
    assert(capacity > 1024); // exercises allocation above the former fixed buffer
    size_t first_capacity = capacity;
    sample.valid = HIPNUC_VALID_ACC;
    assert(canhost_write_sample(file, &sample, 124, &json, &capacity) == 0);
    assert(capacity == first_capacity);
    assert(fflush(file) == 0);
    rewind(file);
    unsigned int lines = 0;
    int character;
    while ((character = fgetc(file)) != EOF) if (character == '\n') ++lines;
    assert(lines == 2);
    free(json);
    assert(fclose(file) == 0);
}
