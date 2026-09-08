/* Run each real main loop through idle -> receive -> stopped -> recovered. */
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static int capture_printf(const char *format, ...);
#define printf capture_printf
#define main example_main
#ifdef TEST_CAN_APPLICATION
#include "../can/USER/main.c"
#else
#include "../serial/USER/main.c"
#endif
#undef main
#undef printf

static jmp_buf completed;
static unsigned step;
static uint32_t now;
static hipnuc_board_stats_t fake_stats;
static char output[4096];
static size_t used;

static int capture_printf(const char *format, ...)
{
    int count;
    va_list args;
    va_start(args, format);
    count = vsnprintf(output + used, sizeof(output) - used, format, args);
    va_end(args);
    if (count < 0 || (size_t)count >= sizeof(output) - used) exit(2);
    used += (size_t)count;
    return count;
}

#ifdef TEST_CAN_APPLICATION
int hipnuc_board_init(uint16_t bitrate, uint8_t source)
{ (void)bitrate; (void)source; return 1; }
#else
void hipnuc_board_init(uint32_t baudrate) { (void)baudrate; }
#endif
uint32_t hipnuc_board_millis(void) { return now; }
const hipnuc_board_stats_t *hipnuc_board_stats(void) { return &fake_stats; }
int hipnuc_board_poll(hipnuc_sample_t *sample)
{
    int receiving;
    if (step == 4) longjmp(completed, 1);
    receiving = step % 2;
    step++;
    now += 2000;
    if (!receiving) {
#ifndef TEST_CAN_APPLICATION
        if (step == 3) fake_stats.hardware_errors++;
#endif
        return 0;
    }
    fake_stats.frames++;
#ifdef TEST_CAN_APPLICATION
    fake_stats.received++;
#else
    fake_stats.bytes += 83;
#endif
    memset(sample, 0, sizeof(*sample));
    sample->valid = HIPNUC_VALID_ACC;
    return 1;
}

static unsigned occurrences(const char *needle)
{
    const char *found = output;
    unsigned count = 0;
    while ((found = strstr(found, needle))) { count++; found += strlen(needle); }
    return count;
}

int main(void)
{
    if (!setjmp(completed)) example_main();
#ifdef TEST_CAN_APPLICATION
    if (occurrences("no CAN frames:") != 2) return 1;
#else
    if (occurrences("no bytes:") != 2) return 1;
    if (occurrences("UART errors:") != 1) return 1;
#endif
    if (occurrences("recovered:") != 2) return 1;
    puts("Application: startup idle, stopped reception and recovery passed");
    return 0;
}
