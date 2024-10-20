#include "hex2bin.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define INITIAL_BUFFER_SIZE (1024 * 1024)  // 1MB initial buffer size

// Convert two hex characters to a byte
static uint8_t hex_to_byte(const char *hex) {
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

// Parse a single line of Intel HEX record
static int parse_hex_record(const char *line, HexRecord *record) {
    if (line[0] != ':') {
        return -1; // Invalid hex record
    }

    record->length = hex_to_byte(line + 1);
    record->address = (hex_to_byte(line + 3) << 8) | hex_to_byte(line + 5);
    record->type = hex_to_byte(line + 7);

    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 1; i < 9 + record->length * 2; i += 2) {
        checksum += hex_to_byte(line + i);
    }
    checksum = ~checksum + 1;

    if (checksum != hex_to_byte(line + 9 + record->length * 2)) {
        return -2; // Checksum error
    }

    for (int i = 0; i < record->length; i++) {
        record->data[i] = hex_to_byte(line + 9 + i * 2);
    }

    return 0;
}

// Convert HEX file to BIN file and return the start address and bin size
int hex_to_bin(const char *hex_file, const char *bin_file, uint32_t *start_address, size_t *bin_size) {
    FILE *in_file = fopen(hex_file, "r");
    if (!in_file) {
        perror("Failed to open input file");
        return -1;
    }

    uint8_t *buffer = (uint8_t *)calloc(INITIAL_BUFFER_SIZE, 1);
    if (!buffer) {
        perror("Memory allocation failed");
        fclose(in_file);
        return -1;
    }

    size_t buffer_size = INITIAL_BUFFER_SIZE;
    uint32_t base_address = 0;
    uint32_t min_address = UINT32_MAX;
    uint32_t max_address = 0;
    uint32_t extended_linear_address = 0;
    char line[MAX_LINE_LENGTH];
    HexRecord record;

    while (fgets(line, sizeof(line), in_file)) {
        // Remove leading/trailing whitespace
        char *start = line;
        char *end = line + strlen(line) - 1;
        while (isspace(*start)) start++;
        while (end > start && isspace(*end)) *end-- = '\0';

        if (parse_hex_record(start, &record) == 0) {
            uint32_t full_address;

            switch (record.type) {
                case 0x00: // Data record
                    full_address = extended_linear_address + base_address + record.address;
                    if (full_address < min_address) min_address = full_address;
                    if (full_address + record.length > max_address) max_address = full_address + record.length;

                    // Ensure buffer is large enough
                    while (full_address + record.length - min_address > buffer_size) {
                        buffer_size *= 2;
                        uint8_t *new_buffer = (uint8_t *)realloc(buffer, buffer_size);
                        if (!new_buffer) {
                            perror("Memory reallocation failed");
                            free(buffer);
                            fclose(in_file);
                            return -1;
                        }
                        buffer = new_buffer;
                        memset(buffer + buffer_size/2, 0, buffer_size/2);
                    }

                    memcpy(buffer + (full_address - min_address), record.data, record.length);
                    break;
                case 0x01: // End of file record
                    break;
                case 0x02: // Extended Segment Address Record
                    base_address = ((uint32_t)record.data[0] << 12) | ((uint32_t)record.data[1] << 4);
                    break;
                case 0x03: // Start Segment Address Record
                    // This record type is ignored for binary file output
                    break;
                case 0x04: // Extended Linear Address Record
                    extended_linear_address = ((uint32_t)record.data[0] << 24) | ((uint32_t)record.data[1] << 16);
                    break;
                case 0x05: // Start Linear Address Record
                    // This record type is ignored for binary file output
                    break;
                default:
                    printf("Warning: Unknown record type 0x%02X\n", record.type);
                    break;
            }
        }
    }

    fclose(in_file);

    if (min_address >= max_address) {
        free(buffer);
        printf("Error: No valid data found\n");
        return -1;
    }

    FILE *out_file = fopen(bin_file, "wb");
    if (!out_file) {
        perror("Failed to create output file");
        free(buffer);
        return -1;
    }

    size_t data_size = max_address - min_address;
    size_t written = fwrite(buffer, 1, data_size, out_file);
    if (written != data_size) {
        perror("Error writing to output file");
        fclose(out_file);
        free(buffer);
        return -1;
    }

    fclose(out_file);
    free(buffer);

    if (start_address) {
        *start_address = min_address;
    }

    if (bin_size) {
        *bin_size = data_size;
    }

    printf("Conversion successful. Wrote %zu bytes, start address: 0x%08X\n", data_size, min_address);
    return 0;
}
