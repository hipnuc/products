/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NMEA 0183 GGA and RMC sentences into hipnuc_data_t (INS products can send
 * them between the binary frames). Pass it the lines that hipnuc_decoder
 * reports: hipnuc_nmea_decode(hipnuc_decoder_line(&dec), &imu).
 *
 * Portable C99: no heap, no stdio, no global state, no locale dependence.
 */

#ifndef HIPNUC_NMEA_H
#define HIPNUC_NMEA_H

#include "hipnuc_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Returns 1 when the line holds a GGA or RMC sentence with a correct
 * checksum (out is cleared, then filled), 0 when it is some other line or
 * sentence, and -1 when a GGA or RMC sentence is malformed or its checksum is
 * wrong. Parsing starts at the last '$' of the line. out is written only on 1. */
int hipnuc_nmea_decode(const char *line, hipnuc_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_NMEA_H */
