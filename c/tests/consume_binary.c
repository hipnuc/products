/* Compile with just hipnuc_dec.c/.h and hipnuc_sample.c/.h copied. */
#include "hipnuc_dec.h"

#ifdef NMEA_DEC_H
#error "Binary decoding must not pull in NMEA"
#endif

int main(void)
{
    hipnuc_raw_t raw = {0};
    hipnuc_sample_t sample;
    return hipnuc_input(&raw, 0) != 0 || hipnuc_sample_from_raw(&raw, &sample) != 0;
}
