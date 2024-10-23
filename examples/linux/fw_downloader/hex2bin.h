#ifndef HEX2BIN_H
#define HEX2BIN_H

#include <stdint.h>
#include <stdlib.h>

typedef struct {
    uint8_t *data;
    size_t size;
    uint32_t start_address;
} binary_data_t;


binary_data_t *hex2bin_convert(const char *hex_file);
void hex2bin_free(binary_data_t *bin_data);

#endif // HEX2BIN_H
