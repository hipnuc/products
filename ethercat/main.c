#include "hi15.h"
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* Edit these parameters for your master and HI15 position. */
#define MASTER_INDEX 0
#define SLAVE_ALIAS 0
#define SLAVE_POS 0
#define VENDOR_ID 0x00131415
#define PRODUCT_CODE 0x00009253
#define ENABLE_DC 1
#define ECAT_FREQUENCY_HZ 1000
#define SYNC0_SHIFT_NS 0
#define UI_HZ 20

#define NSEC_PER_SEC 1000000000L
#define PERIOD_NS (NSEC_PER_SEC / ECAT_FREQUENCY_HZ)

static volatile sig_atomic_t stop_requested;

static void request_stop(int signum)
{
    (void)signum;
    stop_requested = 1;
}

static uint64_t timespec_to_ns(struct timespec t)
{
    return (uint64_t)t.tv_sec * NSEC_PER_SEC + (uint64_t)t.tv_nsec;
}

static struct timespec next_cycle(struct timespec t)
{
    t.tv_nsec += PERIOD_NS;
    if (t.tv_nsec >= NSEC_PER_SEC) {
        t.tv_sec++;
        t.tv_nsec -= NSEC_PER_SEC;
    }
    return t;
}

int main(void)
{
    hi15_ctx_t ctx;
    hi15_txpdo_t tx;
    const hi15_rxpdo_t rx = {0}; /* 0x7000:01 is reserved. */
    struct timespec wakeup, now;
    struct sigaction action = {0};
    unsigned ui_count = 0, sync_count = 0;
    const unsigned ui_div = ECAT_FREQUENCY_HZ / UI_HZ;
    int result = 0;

    action.sa_handler = request_stop;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) || sigaction(SIGTERM, &action, NULL)) {
        perror("Cannot install stop handler");
        return 1;
    }
    if (hi15_init(&ctx, MASTER_INDEX, SLAVE_ALIAS, SLAVE_POS,
                  VENDOR_ID, PRODUCT_CODE, ENABLE_DC, PERIOD_NS, SYNC0_SHIFT_NS)) {
        return 1;
    }
    fprintf(stderr, "HI15: master %u, alias %u, position %u; %u Hz, DC %s. Ctrl-C stops.\n",
            MASTER_INDEX, SLAVE_ALIAS, SLAVE_POS, ECAT_FREQUENCY_HZ,
            ENABLE_DC ? "on" : "off");
    if (clock_gettime(CLOCK_MONOTONIC, &wakeup)) {
        perror("Cannot read monotonic clock");
        result = 1;
        goto done;
    }

    while (!stop_requested) {
        int sleep_error, valid, io_result;
        wakeup = next_cycle(wakeup);
        do {
            sleep_error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);
        } while (sleep_error == EINTR && !stop_requested);
        if (stop_requested) break;
        if (sleep_error) {
            fprintf(stderr, "Cannot wait for EtherCAT cycle: %s\n", strerror(sleep_error));
            result = 1;
            break;
        }
        if (clock_gettime(CLOCK_MONOTONIC, &now)) {
            perror("Cannot read monotonic clock");
            result = 1;
            break;
        }
        if (ENABLE_DC) {
            io_result = ecrt_master_application_time(ctx.master, timespec_to_ns(now));
            if (io_result < 0) {
                fprintf(stderr, "EtherCAT application time failed (%d)\n", io_result);
                result = 1;
                break;
            }
        }

        io_result = hi15_cycle_receive(&ctx);
        if (io_result < 0) {
            fprintf(stderr, "EtherCAT receive/process failed (%d)\n", io_result);
            result = 1;
            break;
        }
        valid = hi15_read_txpdo(&ctx, &tx);
        hi15_write_rxpdo(&ctx, &rx);
        if (ENABLE_DC) {
            if (++sync_count == 100) {
                sync_count = 0;
                io_result = ecrt_master_sync_reference_clock(ctx.master);
                /* The reference clock may be unavailable while the slave starts
                 * or is offline. Keep exchanging PDOs until it is ready. */
                if (io_result < 0 && !(io_result == -ENXIO && !valid)) {
                    fprintf(stderr, "EtherCAT reference clock sync failed (%d)\n", io_result);
                    result = 1;
                    break;
                }
            }
            io_result = ecrt_master_sync_slave_clocks(ctx.master);
            if (io_result < 0) {
                fprintf(stderr, "EtherCAT slave clock sync failed (%d)\n", io_result);
                result = 1;
                break;
            }
        }
        io_result = hi15_cycle_send(&ctx);
        if (io_result < 0) {
            fprintf(stderr, "EtherCAT queue/send failed (%d)\n", io_result);
            result = 1;
            break;
        }

        if (++ui_count >= ui_div) {
            ui_count = 0;
            if (valid) {
                /* The current exchange can repeat a device timestamp. */
                printf("time=%u ms  acc[m/s^2]=%.6f %.6f %.6f  gyr[rad/s]=%.6f %.6f %.6f\n"
                       "quat[WXYZ]=%.6f %.6f %.6f %.6f  temp[degC]=%.3f\n",
                       tx.system_time, tx.acc_x, tx.acc_y, tx.acc_z,
                       tx.gyr_x, tx.gyr_y, tx.gyr_z,
                       tx.qw, tx.qx, tx.qy, tx.qz, tx.temperature);
            } else {
                ec_domain_state_t ds = {0};
                ec_slave_config_state_t ss = {0};
                ecrt_domain_state(ctx.domain, &ds);
                ecrt_slave_config_state(ctx.sc, &ss);
                printf("No valid PDO: WKC=%u, online=%u, operational=%u, AL=0x%02x\n",
                       ds.working_counter, ss.online, ss.operational, ss.al_state);
            }
            if (fflush(stdout)) {
                perror("Cannot write output");
                result = 1;
                break;
            }
        }
        if (clock_gettime(CLOCK_MONOTONIC, &now)) {
            perror("Cannot read monotonic clock");
            result = 1;
            break;
        }
        /* Skip missed periods instead of sending a burst of catch-up cycles. */
        if (timespec_to_ns(now) >= timespec_to_ns(wakeup) + PERIOD_NS) wakeup = now;
    }

done:
    hi15_release(&ctx);
    return result;
}
