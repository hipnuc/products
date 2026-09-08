#ifndef HI15_PDO_H
#define HI15_PDO_H

#include <stdint.h>
#include "ecrt.h"
#if ECRT_VERSION_MAGIC < ECRT_VERSION(1, 6)
#error "This example requires IgH EtherCAT Master 1.6 or newer."
#endif
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float acc_x, acc_y, acc_z; /* m/s^2 */
    float gyr_x, gyr_y, gyr_z; /* rad/s */
    float qw, qx, qy, qz;     /* WXYZ; device-configured reference frame */
    float temperature;       /* degrees C */
    uint32_t system_time;     /* Device uptime in ms; wraps at UINT32_MAX. */
} hi15_txpdo_t;

typedef struct {
    uint32_t rpdo_7000_01; /* Reserved: write zero. */
} hi15_rxpdo_t;

typedef struct {
    /* IgH handles, owned until hi15_release(). */
    ec_master_t *master;
    ec_domain_t *domain;
    ec_slave_config_t *sc;
    uint8_t *domain_pd;

    /* Byte offsets in the fixed HI15 PDO mapping. */
    unsigned int off_rpdo_7000_01;

    unsigned int off_acc_x;
    unsigned int off_acc_y;
    unsigned int off_acc_z;
    unsigned int off_gyr_x;
    unsigned int off_gyr_y;
    unsigned int off_gyr_z;
    unsigned int off_qw;
    unsigned int off_qx;
    unsigned int off_qy;
    unsigned int off_qz;
    unsigned int off_temperature;
    unsigned int off_system_time;
} hi15_ctx_t;

/* Initialize an unused context. Returns 0 on success, -1 on error and releases
 * acquired resources. Errors are printed to stderr by this example helper. */
int hi15_init(hi15_ctx_t *ctx, unsigned master_index, uint16_t alias, uint16_t pos, uint32_t vendor_id, uint32_t product_code,
              int enable_dc, uint32_t sync0_cycle_ns, uint32_t sync0_shift_ns);

/* Receive and process the current exchange before reading PDOs.
 * Returns 0 on success or a negative IgH error. Do not read PDOs after an error. */
int hi15_cycle_receive(hi15_ctx_t *ctx);

/* Returns 1 for a complete exchange with an operational slave, otherwise 0
 * and leaves out unchanged. This does not assert a new sensor sample: consecutive
 * exchanges may carry the same system_time. Call after hi15_cycle_receive(). */
int hi15_read_txpdo(const hi15_ctx_t *ctx, hi15_txpdo_t *out);

/* Write the reserved RxPDO (0x7000:01); pass zero. */
void hi15_write_rxpdo(hi15_ctx_t *ctx, const hi15_rxpdo_t *in);

/* Queue and send, including while waiting for valid input.
 * Returns a nonnegative result on success or a negative IgH error. */
int hi15_cycle_send(hi15_ctx_t *ctx);

/* Release the master. Safe after failed initialization or a previous release. */
void hi15_release(hi15_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif
