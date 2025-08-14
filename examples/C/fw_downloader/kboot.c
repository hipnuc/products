#include "kboot.h"
#include "serial_port.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define BYTES_PER_LINE 32

// Function prototypes
static void crc16(uint16_t *current_crc, const uint8_t *src, uint32_t length_in_bytes);
static int create_framing_packet(uint8_t packet_type, const uint8_t *payload, uint16_t payload_length, uint8_t *output_buffer, size_t output_buffer_size);
static int decode_framing_packet(const uint8_t *packet, size_t packet_size, uint8_t *packet_type, uint8_t *payload);
static bool kboot_send_command(kboot_handle_t *handle, uint32_t command_tag, const uint32_t *params, int param_count, uint32_t *return_params);

/**
 * @brief Print raw data in hexadecimal format
 * @param handle Pointer to kboot_handle_t structure
 * @param prefix Prefix string for the output
 * @param data Pointer to the data to be printed
 * @param length Length of the data in bytes
 */
static void print_raw_data(const kboot_handle_t *handle, const char* prefix, const uint8_t* data, int length) {
    if (!handle->debug_enabled) return;

    printf("%s (%d bytes):\n", prefix, length);

    for (int i = 0; i < length; i += BYTES_PER_LINE) {
        printf("%08X: ", i);

        for (int j = 0; j < BYTES_PER_LINE; j++) {
            if (i + j < length) {
                printf("%02X ", data[i + j]);
            } else {
                printf("   ");
            }

            if (j == 15) {
                printf(" ");
            }
        }

        printf("\n");
    }
}

/**
 * @brief Initialize KBOOT handle
 * @param handle Pointer to kboot_handle_t structure
 * @param fd File descriptor for the serial port
 * @return true if initialization is successful, false otherwise
 */
bool kboot_init(kboot_handle_t *handle, int fd) {
    if (!handle || fd < 0) {
        return false;
    }
    
    handle->fd = fd;
    handle->max_packet_size = KBOOT_MAX_PACKET_SIZE;
    handle->debug_enabled = 0;  // Initialize debug mode to disabled
    return true;
}

/**
 * @brief Set the debug mode for KBOOT operations
 * @param handle Pointer to kboot_handle_t structure
 * @param enable 1 to enable debug mode, 0 to disable
 */
void kboot_set_debug(kboot_handle_t *handle, int enable) {
    if (handle) {
        handle->debug_enabled = enable;
    }
}

/**
 * @brief Ping the bootloader
 * @param handle Pointer to kboot_handle_t structure
 * @return true if ping is successful, false otherwise
 */
bool kboot_ping(kboot_handle_t *handle) {
    uint8_t ping_packet[] = {kFramingPacketStartByte, kFramingPacketType_Ping};
    uint8_t response[10];
    int response_length;

    response_length = serial_send_then_recv(handle->fd, ping_packet, sizeof(ping_packet), response, sizeof(response), 50);
    
    print_raw_data(handle, "Ping packet sent", ping_packet, sizeof(ping_packet));
    print_raw_data(handle, "Ping response received", response, response_length);

    return (response_length >= 10 && response[0] == kFramingPacketStartByte && response[1] == kFramingPacketType_PingResponse);
}

/**
 * @brief Create a framing packet
 * @param packet_type Type of the packet
 * @param payload Pointer to the payload data
 * @param payload_length Length of the payload
 * @param output_buffer Buffer to store the created packet
 * @param output_buffer_size Size of the output buffer
 * @return Length of the created packet, or -1 if failed
 */
static int create_framing_packet(uint8_t packet_type, const uint8_t *payload, uint16_t payload_length, uint8_t *output_buffer, size_t output_buffer_size) {
    if (output_buffer_size < payload_length + 6) {
        return -1;
    }

    output_buffer[0] = kFramingPacketStartByte;
    output_buffer[1] = packet_type;
    output_buffer[2] = payload_length & 0xFF;
    output_buffer[3] = (payload_length >> 8) & 0xFF;

    memcpy(output_buffer + 6, payload, payload_length);

    uint16_t crc = 0;
    crc16(&crc, output_buffer, 4);
    crc16(&crc, output_buffer + 6, payload_length);
    output_buffer[4] = crc & 0xFF;
    output_buffer[5] = (crc >> 8) & 0xFF;

    return payload_length + 6;
}

/**
 * @brief Decode a framing packet
 * @param packet Pointer to the received packet
 * @param packet_size Size of the received packet
 * @param packet_type Pointer to store the decoded packet type
 * @param payload Pointer to store the decoded payload
 * @return Length of the decoded payload, or -1 if failed
 */
static int decode_framing_packet(const uint8_t *packet, size_t packet_size, uint8_t *packet_type, uint8_t *payload) {
    if (packet_size < 6 || packet[0] != kFramingPacketStartByte) {
        return -1;
    }

    *packet_type = packet[1];
    uint16_t payload_length = packet[2] | (packet[3] << 8);

    if (packet_size < payload_length + 6) {
        return -1;
    }

    uint16_t received_crc = packet[4] | (packet[5] << 8);
    uint16_t calculated_crc = 0;
    crc16(&calculated_crc, packet, 4);
    crc16(&calculated_crc, packet + 6, payload_length);

    if (received_crc != calculated_crc) {
        return -1;
    }

    memcpy(payload, packet + 6, payload_length);

    return payload_length;
}

/**
 * @brief Send a command to the bootloader
 * @param handle Pointer to kboot_handle_t structure
 * @param command_tag Command tag
 * @param params Pointer to the parameters
 * @param param_count Number of parameters
 * @param return_params Pointer to store the return parameters
 * @return true if command is sent successfully, false otherwise
 */
static bool kboot_send_command(kboot_handle_t *handle, uint32_t command_tag, const uint32_t *params, int param_count, uint32_t *return_params) {
    uint8_t raw_tx_buffer[KBOOT_MAX_PACKET_SIZE], raw_rx_buffer[KBOOT_MAX_PACKET_SIZE];
    uint8_t payload[KBOOT_MAX_PACKET_SIZE], response_packet_type;
    int raw_len, response_payload_len;

    payload[0] = command_tag;
    payload[1] = 0;  // flags
    payload[2] = 0;  // reserved
    payload[3] = param_count;

    for (int i = 0; i < param_count; i++) {
        *(uint32_t *)(payload + (i + 1) * sizeof(uint32_t)) = params[i];
    }

    raw_len = create_framing_packet(kFramingPacketType_Command, payload, (param_count + 1) * sizeof(uint32_t), raw_tx_buffer, sizeof(raw_tx_buffer));

    print_raw_data(handle, "TX: ", raw_tx_buffer, raw_len);

    raw_len = serial_send_then_recv(handle->fd, raw_tx_buffer, raw_len, raw_rx_buffer, 2 + 6 + (3) * 4, KBOOT_CMD_TIMEOUT_MS);

    print_raw_data(handle, "RX: ", raw_rx_buffer, raw_len);

    response_payload_len = decode_framing_packet(raw_rx_buffer + 2, raw_len - 2, &response_packet_type, payload);

    if (return_params && response_payload_len >= 8) {
        *return_params = *(uint32_t *)(payload + 8);
    }

    return (response_payload_len > 0);
}

/**
 * @brief Get a property from the bootloader
 * @param handle Pointer to kboot_handle_t structure
 * @param property_tag Property tag to get
 * @param value Pointer to store the property value
 * @return true if property is retrieved successfully, false otherwise
 */
bool kboot_get_property(kboot_handle_t *handle, uint32_t property_tag, uint32_t *value) {
    uint32_t params[2] = {property_tag, 0};
    return kboot_send_command(handle, kCommandTag_GetProperty, params, 2, value);
}

/**
 * @brief Erase a region of flash memory
 * @param handle Pointer to kboot_handle_t structure
 * @param addr Start address of the region to erase
 * @param len Length of the region to erase
 * @return true if erase is successful, false otherwise
 */
bool kboot_flash_erase_region(kboot_handle_t *handle, uint32_t addr, uint32_t len) {
    uint32_t params[2] = {addr, len};
    return kboot_send_command(handle, kCommandTag_FlashEraseRegion, params, 2, NULL);
}

/**
 * @brief Prepare to write to flash memory
 * @param handle Pointer to kboot_handle_t structure
 * @param addr Start address to write
 * @param len Length of data to write
 * @return true if preparation is successful, false otherwise
 */
bool kboot_flash_write_memory(kboot_handle_t *handle, uint32_t addr, uint32_t len) {
    uint32_t params[2] = {addr, len};
    return kboot_send_command(handle, kCommandTag_WriteMemory, params, 2, NULL);
}

/**
 * @brief Send data to the bootloader
 * @param handle Pointer to kboot_handle_t structure
 * @param buf Pointer to the data buffer
 * @param len Length of the data
 * @return true if data is sent successfully, false otherwise
 */
bool kboot_send_data(kboot_handle_t *handle, const void *buf, uint32_t len) {
    uint8_t raw_tx_buffer[KBOOT_MAX_PACKET_SIZE];
    uint8_t raw_rx_buffer[2];
    int raw_len;

    raw_len = create_framing_packet(kFramingPacketType_Data, buf, len, raw_tx_buffer, sizeof(raw_tx_buffer));
    if (raw_len < 0) {
        return false;
    }

    raw_len = serial_send_then_recv(handle->fd, raw_tx_buffer, raw_len, raw_rx_buffer, 2, KBOOT_CMD_TIMEOUT_MS);

    return (raw_len >= 2 && raw_rx_buffer[0] == kFramingPacketStartByte && raw_rx_buffer[1] == kFramingPacketType_Ack);
}

/**
 * @brief Reset the target device
 * @param handle Pointer to kboot_handle_t structure
 * @return true if reset command is sent successfully, false otherwise
 */
bool kboot_reset(kboot_handle_t *handle) {
    return kboot_send_command(handle, kCommandTag_Reset, NULL, 0, NULL);
}

/**
 * @brief Calculate CRC16
 * @param currentCrc Pointer to the current CRC value
 * @param src Pointer to the source data
 * @param lengthInBytes Length of the data in bytes
 */
static void crc16(uint16_t *currentCrc, const uint8_t *src, uint32_t lengthInBytes) {
    uint32_t crc = *currentCrc;
    for (uint32_t j = 0; j < lengthInBytes; ++j) {
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    *currentCrc = crc & 0xFFFF;
}
