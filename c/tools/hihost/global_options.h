/* Options shared by every hihost command. */
#ifndef GLOBAL_OPTIONS_H
#define GLOBAL_OPTIONS_H

typedef struct {
    const char *port_name;          /* /dev/ttyUSB0 or ttyUSB0 */
    int baud_rate;
    const char *record_raw_file;    /* raw serial bytes, binary */
    const char *record_json_file;   /* one JSON object per line */
} GlobalOptions;

#endif
