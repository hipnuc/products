#include "hi15.h"
#include <stdio.h>
#include <string.h>

static float u32_to_f32(uint32_t u)
{
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

static int register_pdos(hi15_ctx_t *ctx, uint16_t alias, uint16_t pos, uint32_t vendor_id, uint32_t product_code)
{
    const ec_pdo_entry_reg_t regs[] = {
        {alias, pos, vendor_id, product_code, 0x7000, 0x01, &ctx->off_rpdo_7000_01, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x01, &ctx->off_acc_x, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x02, &ctx->off_acc_y, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x03, &ctx->off_acc_z, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x04, &ctx->off_gyr_x, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x05, &ctx->off_gyr_y, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x06, &ctx->off_gyr_z, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x07, &ctx->off_qw, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x08, &ctx->off_qx, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x09, &ctx->off_qy, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x0a, &ctx->off_qz, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x0b, &ctx->off_temperature, NULL},
        {alias, pos, vendor_id, product_code, 0x6000, 0x0c, &ctx->off_system_time, NULL},
        {0}
    };

    if (ecrt_domain_reg_pdo_entry_list(ctx->domain, regs)) {
        fprintf(stderr, "ERROR: ecrt_domain_reg_pdo_entry_list failed (PDO map mismatch?)\n");
        return -1;
    }
    return 0;
}


int hi15_init(hi15_ctx_t *ctx, unsigned master_index, uint16_t alias, uint16_t pos, uint32_t vendor_id, 
                uint32_t product_code, int enable_dc, uint32_t sync0_cycle_ns, uint32_t sync0_shift_ns)
{
    memset(ctx, 0, sizeof(*ctx));

    ctx->master = ecrt_request_master(master_index);
    if (!ctx->master) {
        fprintf(stderr, "ERROR: request master %u failed\n", master_index);
        goto fail;
    }

    ctx->domain = ecrt_master_create_domain(ctx->master);
    if (!ctx->domain) {
        fprintf(stderr, "ERROR: create domain failed\n");
        goto fail;
    }

    ctx->sc = ecrt_master_slave_config(ctx->master, alias, pos, vendor_id, product_code);
    if (!ctx->sc) {
        fprintf(stderr, "ERROR: slave config failed (alias=%u pos=%u vendor=0x%08X product=0x%08X)\n",
                alias, pos, vendor_id, product_code);
        goto fail;
    }
 
    if (register_pdos(ctx, alias, pos, vendor_id, product_code) != 0) {
        goto fail;
    }

    if (enable_dc) {
        if (ecrt_slave_config_dc(ctx->sc, 0x0300,
                                 sync0_cycle_ns, sync0_shift_ns,
                                 0, 0)) {
            fprintf(stderr, "ERROR: ecrt_slave_config_dc failed\n");
            goto fail;
        }
    }

    if (ecrt_master_activate(ctx->master)) {
        fprintf(stderr, "ERROR: master activate failed\n");
        goto fail;
    }

    ctx->domain_pd = ecrt_domain_data(ctx->domain);
    if (!ctx->domain_pd) {
        fprintf(stderr, "ERROR: domain data is NULL\n");
        goto fail;
    }

    return 0;

fail:
    hi15_release(ctx);
    return -1;
}

int hi15_cycle_receive(hi15_ctx_t *ctx)
{
    int result = ecrt_master_receive(ctx->master);
    return result < 0 ? result : ecrt_domain_process(ctx->domain);
}

int hi15_read_txpdo(const hi15_ctx_t *ctx, hi15_txpdo_t *out)
{
    ec_domain_state_t domain_state = {0};
    ec_slave_config_state_t slave_state = {0};
    if (ecrt_domain_state(ctx->domain, &domain_state) != 0 ||
        domain_state.wc_state != EC_WC_COMPLETE ||
        ecrt_slave_config_state(ctx->sc, &slave_state) != 0 || !slave_state.operational) {
        return 0;
    }
    out->acc_x = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_acc_x));
    out->acc_y = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_acc_y));
    out->acc_z = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_acc_z));

    out->gyr_x = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_gyr_x));
    out->gyr_y = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_gyr_y));
    out->gyr_z = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_gyr_z));

    out->qw = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_qw));
    out->qx = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_qx));
    out->qy = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_qy));
    out->qz = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_qz));

    out->temperature = u32_to_f32(EC_READ_U32(ctx->domain_pd + ctx->off_temperature));
    out->system_time = EC_READ_U32(ctx->domain_pd + ctx->off_system_time);
    return 1;
}

void hi15_write_rxpdo(hi15_ctx_t *ctx, const hi15_rxpdo_t *in)
{
    EC_WRITE_U32(ctx->domain_pd + ctx->off_rpdo_7000_01, in->rpdo_7000_01);
}

int hi15_cycle_send(hi15_ctx_t *ctx)
{
    int result = ecrt_domain_queue(ctx->domain);
    return result < 0 ? result : ecrt_master_send(ctx->master);
}

void hi15_release(hi15_ctx_t *ctx)
{
    if (ctx->master) {
        ecrt_release_master(ctx->master);
    }
    memset(ctx, 0, sizeof(*ctx));
}
