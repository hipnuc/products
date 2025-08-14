// hex2bin.c

#include "hex2bin.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define MAX_LINE_LENGTH 256
#define INITIAL_BUFFER_SIZE (1024 * 1024)  // 1MB initial buffer size

typedef struct {
    uint8_t length;
    uint16_t address;
    uint8_t type;
    uint8_t data[256];
} hex_record_t;

static uint8_t hex2byte(const char *hex) {
    uint8_t byte = 0;
    for (int i = 0; i < 2; i++) {
        byte <<= 4;
        if (hex[i] >= '0' && hex[i] <= '9') {
            byte |= hex[i] - '0';
        } else if (hex[i] >= 'A' && hex[i] <= 'F') {
            byte |= hex[i] - 'A' + 10;
        } else if (hex[i] >= 'a' && hex[i] <= 'f') {
            byte |= hex[i] - 'a' + 10;
        }
    }
    return byte;
}

static int hex_parse_line(const char *line, hex_record_t *record) {
    if (line[0] != ':') {
        return -1; // Invalid hex record
    }

    record->length = hex2byte(line + 1);
    record->address = (hex2byte(line + 3) << 8) | hex2byte(line + 5);
    record->type = hex2byte(line + 7);

    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 1; i < 9 + record->length * 2; i += 2) {
        checksum += hex2byte(line + i);
    }
    checksum = ~checksum + 1;

    if (checksum != hex2byte(line + 9 + record->length * 2)) {
        return -2; // Checksum error
    }

    for (int i = 0; i < record->length; i++) {
        record->data[i] = hex2byte(line + 9 + i * 2);
    }

    return 0;
}

static int hex_process_record(const hex_record_t *record, binary_data_t *bin_data) {
    static uint32_t extended_linear_address = 0;
    uint32_t full_address = extended_linear_address + record->address;

    switch (record->type) {
        case 0x00: // Data record
            if (full_address + record->length > bin_data->size) {
                size_t new_size = full_address + record->length;
                uint8_t *new_data = realloc(bin_data->data, new_size);
                if (!new_data) return -1;
                memset(new_data + bin_data->size, 0xFF, new_size - bin_data->size);
                bin_data->data = new_data;
                bin_data->size = new_size;
            }
            memcpy(bin_data->data + full_address, record->data, record->length);
            if (full_address < bin_data->start_address) bin_data->start_address = full_address;
            break;
        case 0x01: // End of file record
            break;
        case 0x02: // Extended Segment Address Record
            extended_linear_address = ((uint32_t)record->data[0] << 12) | ((uint32_t)record->data[1] << 4);
            break;
        case 0x04: // Extended Linear Address Record
            extended_linear_address = ((uint32_t)record->data[0] << 24) | ((uint32_t)record->data[1] << 16);
            break;
        default:
            return -1; // Unknown record type
    }
    return 0;
}

binary_data_t *hex2bin_convert(const char *hex_file) {
    FILE *in_file = fopen(hex_file, "r");
    if (!in_file) return NULL;

    binary_data_t *bin_data = calloc(1, sizeof(binary_data_t));
    if (!bin_data) {
        fclose(in_file);
        return NULL;
    }

    bin_data->data = malloc(INITIAL_BUFFER_SIZE);
    bin_data->size = INITIAL_BUFFER_SIZE;
    bin_data->start_address = UINT32_MAX;

    if (!bin_data->data) {
        free(bin_data);
        fclose(in_file);
        return NULL;
    }

    char line[MAX_LINE_LENGTH];
    hex_record_t record;

    while (fgets(line, sizeof(line), in_file)) {
        char *start = line;
        char *end = line + strlen(line) - 1;
        while (isspace(*start)) start++;
        while (end > start && isspace(*end)) *end-- = '\0';

        if (hex_parse_line(start, &record) == 0) {
            if (hex_process_record(&record, bin_data) != 0) {
                hex2bin_free(bin_data);
                fclose(in_file);
                return NULL;
            }
        }
    }

    fclose(in_file);

    // Trim unused space
    if (bin_data->size > bin_data->start_address) {
        size_t actual_size = bin_data->size - bin_data->start_address;
        memmove(bin_data->data, bin_data->data + bin_data->start_address, actual_size);
        bin_data->data = realloc(bin_data->data, actual_size);
        bin_data->size = actual_size;
    } else {
        hex2bin_free(bin_data);
        return NULL;
    }

    return bin_data;
}

void hex2bin_free(binary_data_t *bin_data) {
    if (bin_data) {
        free(bin_data->data);
        free(bin_data);
    }
}
