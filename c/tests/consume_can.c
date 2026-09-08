/* Compile with only the CAN and sample files from the public copy list. */
#include "hipnuc_j1939.h"

#if defined(HIPNUC_DEC_H) || defined(NMEA_DEC_H)
#error "CAN decoding must not pull in serial protocols"
#endif

int main(void)
{
    hipnuc_can_frame_t frame = {0};
    hipnuc_sample_t sample;
    frame.id = hipnuc_j1939_data_id(HIPNUC_J1939_PGN_ACC, 8);
    frame.is_extended = 1;
    frame.len = 6;
    frame.data[5] = 8; /* z = 2048 -> 9.8 m/s^2 */
    return hipnuc_j1939_parse(&frame, &sample, 0) <= 0 ||
           sample.valid != (HIPNUC_VALID_NODE_ID | HIPNUC_VALID_ACC) || sample.acc[2] != 9.8f;
}
