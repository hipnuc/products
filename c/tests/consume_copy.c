/* A customer project that copied only the serial decoding files. */
#include <stdio.h>
#include <string.h>

#include "hipnuc_dec.h"
#include "hipnuc_sample.h"
#include "nmea_dec.h"

int main(void)
{
    hipnuc_raw_t raw;
    nmea_raw_t nmea;
    hipnuc_sample_t sample;
    uint8_t bytes[] = { 0x5A, 0xA5, 0x00, 0x00, 0x00, 0x00 };   /* zero length: rejected */
    size_t i;
    int last = 0;

    memset(&raw, 0, sizeof(raw));
    memset(&nmea, 0, sizeof(nmea));
    for (i = 0; i < sizeof(bytes); ++i) last = hipnuc_input(&raw, bytes[i]);
    if (last != -1) return 1;
    if (nmea_input(&nmea, '$') != 0) return 1;
    hipnuc_sample_clear(&sample);
    if (hipnuc_sample_from_raw(&raw, &sample) != 0) return 1;
    printf("consume_copy_c: ok\n");
    return 0;
}
