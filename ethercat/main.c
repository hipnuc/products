#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "hi15.h"

#define MASTER_INDEX 0
#define SLAVE_ALIAS  0
#define SLAVE_POS    0
#define VENDOR_ID    0x00131415
#define PRODUCT_CODE 0x00009253

// ===== DC频率=====
#define ECAT_FREQUENCY_HZ 1000

// 界面刷新频率
#define UI_HZ 20

#define NSEC_PER_SEC 1000000000L
#define PERIOD_NS (NSEC_PER_SEC / ECAT_FREQUENCY_HZ)

#define CLOCK_TO_USE CLOCK_MONOTONIC

static inline uint64_t timespec_to_ns(struct timespec t)
{
    return (uint64_t)t.tv_sec * (uint64_t)NSEC_PER_SEC + (uint64_t)t.tv_nsec;
}

static inline struct timespec timespec_add_ns(struct timespec t, long ns)
{
    t.tv_nsec += ns;
    while (t.tv_nsec >= NSEC_PER_SEC) {
        t.tv_sec += 1;
        t.tv_nsec -= NSEC_PER_SEC;
    }
    return t;
}

static const char* al_state_str(uint8_t s)
{
    switch (s) {
        case 0x01: return "INIT";
        case 0x02: return "PREOP";
        case 0x04: return "SAFEOP";
        case 0x08: return "OP";
        default:   return "UNKNOWN";
    }
}

int main()
{
    hi15_ctx_t ctx;
    hi15_txpdo_t tx;
    hi15_rxpdo_t rx;
 
    const int enable_dc = 1;
    const uint32_t sync0_cycle_ns = (uint32_t)PERIOD_NS;
    const uint32_t sync0_shift_ns = 0;

    if (hi15_init(&ctx,
                  MASTER_INDEX,
                  SLAVE_ALIAS, SLAVE_POS,
                  VENDOR_ID, PRODUCT_CODE,
                  enable_dc,
                  sync0_cycle_ns,
                  sync0_shift_ns) != 0) {
        return 1;
    }
 
    printf("\033[2J\033[H"); 
    printf("\033[?25l");
    fflush(stdout);
 
    struct timespec wakeup;
    clock_gettime(CLOCK_TO_USE, &wakeup);
 
    const uint32_t ui_div = (UI_HZ > 0) ? (ECAT_FREQUENCY_HZ / UI_HZ) : ECAT_FREQUENCY_HZ;
    uint32_t ui_count = 0;
 
    uint32_t sync_ref_div = 100;  
    uint32_t sync_ref_cnt = 0;

    uint32_t counter = 0;

    while (1) { 
        wakeup = timespec_add_ns(wakeup, PERIOD_NS);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeup, NULL);
 
        hi15_cycle_receive(&ctx);
 
        hi15_read_txpdo(&ctx, &tx);
 
        static uint32_t out = 0;
        rx.rpdo_7000_01 = out++;
        hi15_write_rxpdo(&ctx, &rx);
 
        struct timespec now;
        clock_gettime(CLOCK_TO_USE, &now);
        ecrt_master_application_time(ctx.master, timespec_to_ns(now));
 
        if (++sync_ref_cnt >= sync_ref_div) {
            sync_ref_cnt = 0;
            ecrt_master_sync_reference_clock(ctx.master);
        }
 
        ecrt_master_sync_slave_clocks(ctx.master);
 
        hi15_cycle_send(&ctx);
 
        if (++ui_count >= (ui_div ? ui_div : 1)) {
            ui_count = 0;

            ec_slave_config_state_t ss;
            ecrt_slave_config_state(ctx.sc, &ss);

            printf("\033[H");
            printf("ECAT: %d Hz  (DC=%s, Sync0=%u ns)   UI: %d Hz   counter=%u\n",
                   ECAT_FREQUENCY_HZ, enable_dc ? "ON" : "OFF",
                   (unsigned)sync0_cycle_ns, UI_HZ, counter++);

            printf("Slave: alias=%u pos=%u  vendor=0x%08X product=0x%08X\n",
                   SLAVE_ALIAS, SLAVE_POS, VENDOR_ID, PRODUCT_CODE);

            printf("State: %-6s (0x%02X)  online=%u  operational=%u\n",
                   al_state_str(ss.al_state), ss.al_state, ss.online, ss.operational);

            printf("acc:  %+10.6f  %+10.6f  %+10.6f\n", tx.acc_x, tx.acc_y, tx.acc_z);
            printf("gyr:  %+10.6f  %+10.6f  %+10.6f\n", tx.gyr_x, tx.gyr_y, tx.gyr_z);
            printf("temp: %+10.6f\n", tx.temperature);
            printf("system_time: %u\n", (unsigned)tx.system_time);
            printf("rpdo_7000:01 (u32): %u\n", (unsigned)rx.rpdo_7000_01);

            fflush(stdout);
        }
    }

    hi15_release(&ctx);
    return 0;
}
