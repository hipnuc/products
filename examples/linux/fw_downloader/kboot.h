#ifndef KBOOT_H
#define KBOOT_H

#include <stdint.h>
#include <stdbool.h>

#define KBOOT_CMD_TIMEOUT_MS    (5000)
#define KBOOT_MAX_PACKET_SIZE   (512)

// Packet types
typedef enum {
    kFramingPacketStartByte         = 0x5A,
    kFramingPacketType_Ack          = 0xA1,
    kFramingPacketType_Nak          = 0xA2,
    kFramingPacketType_AckAbort     = 0xA3,
    kFramingPacketType_Command      = 0xA4,
    kFramingPacketType_Data         = 0xA5,
    kFramingPacketType_Ping         = 0xA6,
    kFramingPacketType_PingResponse = 0xA7
} kboot_packet_type_t;

// Command tags
typedef enum {
    kCommandTag_GenericResponse             = 0xa0,
    kCommandTag_FlashEraseAll               = 0x01,
    kCommandTag_FlashEraseRegion            = 0x02,
    kCommandTag_ReadMemory                  = 0x03,
    kCommandTag_ReadMemoryResponse          = 0xa3,
    kCommandTag_WriteMemory                 = 0x04,
    kCommandTag_FillMemory                  = 0x05,
    kCommandTag_FlashSecurityDisable        = 0x06,
    kCommandTag_GetProperty                 = 0x07,
    kCommandTag_GetPropertyResponse         = 0xa7,
    kCommandTag_ReceiveSbFile               = 0x08,
    kCommandTag_Execute                     = 0x09,
    kCommandTag_Call                        = 0x0a,
    kCommandTag_Reset                       = 0x0b,
    kCommandTag_SetProperty                 = 0x0c,
    kCommandTag_FlashEraseAllUnsecure       = 0x0d,
    kCommandTag_FlashProgramOnce            = 0x0e,
    kCommandTag_FlashReadOnce               = 0x0f,
    kCommandTag_FlashReadOnceResponse       = 0xaf,
    kCommandTag_FlashReadResource           = 0x10,
    kCommandTag_FlashReadResourceResponse   = 0xb0,
    kCommandTag_ConfigureQuadSpi            = 0x11
} kboot_command_tag_t;

typedef struct {
    int fd;
    uint32_t max_packet_size;
    uint32_t flash_size;
    uint32_t bl_version;
    uint32_t app_version;
    uint32_t flash_sector_size;
    int debug_enabled;  // Add this line
} kboot_handle_t;

bool kboot_init(kboot_handle_t *handle, int fd);
void kboot_set_debug(kboot_handle_t *handle, int enable);
bool kboot_ping(kboot_handle_t *handle);
bool kboot_get_property(kboot_handle_t *handle, uint32_t property_tag, uint32_t *value);
bool kboot_flash_erase_region(kboot_handle_t *handle, uint32_t addr, uint32_t len);
bool kboot_flash_write_memory(kboot_handle_t *handle, uint32_t addr, uint32_t len);
bool kboot_send_data(kboot_handle_t *handle, const void *buf, uint32_t len);
bool kboot_reset(kboot_handle_t *handle);

#endif // KBOOT_H
