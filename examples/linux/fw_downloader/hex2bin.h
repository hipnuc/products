#ifndef HEX2BIN_H
#define HEX2BIN_H

#include <stdint.h>
#include <stdio.h>

#define MAX_LINE_LENGTH 256
#define MAX_DATA_LENGTH 16

typedef struct {
    uint8_t data[MAX_DATA_LENGTH];
    uint32_t address;
    uint8_t length;
    uint8_t type;
} HexRecord;

int hex_to_bin(const char *hex_file, const char *bin_file, uint32_t *start_address, size_t *bin_size);



#endif // HEX2BIN_H
