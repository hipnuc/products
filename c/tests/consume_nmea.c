/* Compile with just nmea_dec.c/.h and hipnuc_sample.c/.h copied. */
#include "nmea_dec.h"

#ifdef HIPNUC_DEC_H
#error "NMEA decoding must not pull in binary decoding"
#endif

int main(void)
{
    nmea_raw_t raw = {0};
    hipnuc_sample_t sample;
    return nmea_input(&raw, 'x') != 0 || hipnuc_sample_from_nmea(&raw, &sample) != 0;
}
