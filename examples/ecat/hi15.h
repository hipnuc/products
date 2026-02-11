#ifndef HI15_PDO_H
#define HI15_PDO_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/resource.h>

#include "ecrt.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
    float qw, qx, qy, qz;
    float temperature;
    uint32_t system_time;
} hi15_txpdo_t;

typedef struct {
    uint32_t rpdo_7000_01;
} hi15_rxpdo_t;

typedef struct {
    // IGH handles
    ec_master_t *master;
    ec_domain_t *domain;
    ec_slave_config_t *sc;
    uint8_t *domain_pd;

    // offsets
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

// 初始化（申请 master、创建 domain、注册 PDO、activate）
int hi15_init(hi15_ctx_t *ctx, unsigned master_index, uint16_t alias, uint16_t pos, uint32_t vendor_id, uint32_t product_code,
              int enable_dc, uint32_t sync0_cycle_ns, uint32_t sync0_shift_ns);

// 一次收发周期：receive + process（读前必须调用）
void hi15_cycle_receive(hi15_ctx_t *ctx);

// 读 TxPDO 到结构体
void hi15_read_txpdo(const hi15_ctx_t *ctx, hi15_txpdo_t *out);

// 写 RxPDO（0x7000:01）
void hi15_write_rxpdo(hi15_ctx_t *ctx, const hi15_rxpdo_t *in);

// 发送：queue + send（写后必须调用）
void hi15_cycle_send(hi15_ctx_t *ctx);

// 释放 master
void hi15_release(hi15_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif


