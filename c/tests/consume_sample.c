/* A sample-only application must not depend on a wire protocol header. */
#include "hipnuc_sample.h"

#if defined(HIPNUC_DEC_H) || defined(NMEA_DEC_H) || defined(HIPNUC_J1939_H)
#error "The sample header must be independent of protocol headers"
#endif

int main(void)
{
    hipnuc_sample_t sample;
    hipnuc_sample_clear(&sample);
    hipnuc_sample_set_status(&sample, HIPNUC_STATUS_ATT_CONV);
    return sample.valid != HIPNUC_VALID_STATUS || sample.attitude_converged != 0;
}
