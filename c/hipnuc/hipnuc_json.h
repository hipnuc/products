/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * JSON formatting of hipnuc_sample_t (optional; uses stdio).
 *
 * Output is one JSON object without a trailing newline, in SI units, with
 * the same key names as the Python SDK: acceleration_m_s2,
 * angular_velocity_rad_s, magnetic_field_t, euler_rad, quaternion_wxyz,
 * heading_rad, pressure_pa, temperature_c, device_time_us, utc,
 * longitude_deg, latitude_deg, altitude_msl_m, velocity_enu_m_s, ... Only
 * fields whose availability bit is set are emitted. Partial euler_rad arrays
 * use null for absent components. INS and raw GNSS coordinates are separate.
 * Numeric output always uses a decimal point, without changing the locale.
 */

#ifndef HIPNUC_JSON_H
#define HIPNUC_JSON_H

#include <stddef.h>

#include "hipnuc_sample.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Format a sample as JSON.
 *
 * @param buf output buffer, or NULL with size 0 to query the required length
 * @param size capacity including the terminating NUL
 * @return number of characters written (excluding NUL), or -1 when the
 *         buffer is too small, a value is not finite, or arguments are
 *         invalid. A nonempty buffer is cleared on failure; no partial JSON
 *         is ever returned.
 */
int hipnuc_json_sample(const hipnuc_sample_t *sample, char *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_JSON_H */
