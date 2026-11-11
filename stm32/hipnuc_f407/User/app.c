/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC STM32F407 example for the GK-407VE core board: reads a HiPNUC IMU and
 * prints a summary every 2 s on the USART1 console. main.c calls app_start()
 * and app_poll(); CubeMX sets up the clocks and peripherals before that.
 */

#include "app.h"

#include <stdio.h>

#include "main.h"

/* The IMU's interface:
 *   IMU_UART: USART2, PA2 TX -> IMU RX, PA3 RX <- IMU TX (3.3 V logic)
 *   IMU_CAN:  CAN1, PB9 TX -> transceiver TXD, PB8 RX <- transceiver RXD
 *             (a 3.3 V CAN transceiver; its CAN_H/CAN_L to the IMU's)
 * The baud rate and the CAN bit timing are set in CubeMX (USART2, CAN1). */
#define IMU_UART 1
#define IMU_CAN  2
#define IMU_INTERFACE IMU_UART

static uint32_t last;   /* time of the last summary */

#if IMU_INTERFACE == IMU_UART

#include "hipnuc_decoder.h"
#include "imu_uart.h"

static hipnuc_decoder_t decoder;   /* zero-filled: ready to use */
static hipnuc_data_t imu;          /* the last decoded message */
static uint32_t frames, bytes;     /* the decoder's counts at the last summary */
static int asked;

/* HAL calls this for every UART; it hands the IMU UART's bytes on. If your
 * project already has this callback, add the call to yours. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    imu_uart_rx_complete(huart);
}

/* Text lines between the frames: here, the reply to LOG VERSION. */
static void on_line(const char *line)
{
    char value[48];
    if (hipnuc_line_value(line, "PNAME", value, sizeof(value))) printf("IMU: %s\n", value);
    if (hipnuc_line_value(line, "APP_VER", value, sizeof(value))) printf("IMU firmware: %s\n", value);
}

static void print_summary(void)
{
    printf("%lu Hz", (unsigned long)((decoder.frames - frames) / 2U));
    if (decoder.frames != frames && (imu.valid & HIPNUC_VALID_ROLL_PITCH)) {
        printf("  roll %.1f pitch %.1f", imu.roll * HIPNUC_RAD2DEG, imu.pitch * HIPNUC_RAD2DEG);
    }
    if (decoder.frames != frames && (imu.valid & HIPNUC_VALID_YAW)) printf(" yaw %.1f", imu.yaw * HIPNUC_RAD2DEG);
    printf(" | frames %lu crc %lu err %lu dropped %lu\n", (unsigned long)decoder.frames,
           (unsigned long)decoder.crc_errors, (unsigned long)imu_uart_errors, (unsigned long)imu_uart_dropped);
    if (decoder.bytes == bytes) {
        printf("hint: no bytes from the IMU: check its power, GND, IMU TX -> PA3 and that its output is on\n");
    } else if (decoder.frames == frames && decoder.unsupported > 0) {
        printf("hint: frames this example does not decode: check the output setting (LOG HI83 MAP)\n");
    } else if (decoder.frames == frames) {
        printf("hint: bytes but no frames: check that the IMU runs at %lu baud and outputs HI91 or HI83\n",
               (unsigned long)IMU_UART_HANDLE->Init.BaudRate);
    }
}

void app_start(void)
{
    printf("\nHiPNUC STM32F407 example: IMU on USART2 at %lu baud, receive by %s\n",
           (unsigned long)IMU_UART_HANDLE->Init.BaudRate, IMU_UART_DMA ? "DMA" : "interrupt");
    imu_uart_start();
}

void app_poll(void)
{
    uint8_t buf[64];
    const uint8_t *p = buf;
    size_t n = imu_uart_read(buf, sizeof(buf));
    int ev;

    while ((ev = hipnuc_decoder_feed(&decoder, &p, &n, &imu)) != 0) {
        if (ev == HIPNUC_EV_LINE) {
            on_line(hipnuc_decoder_line(&decoder));
        } else if (!asked) {   /* data flows: ask once who sends it (read only, nothing changes) */
            asked = 1;
            imu_uart_write("\r\nLOG VERSION\r\n");
        }
        /* use imu here: imu.acc, imu.gyr, imu.quat, ... (check imu.valid first) */
    }

    if (HAL_GetTick() - last >= 2000U) {
        last += 2000U;
        print_summary();
        frames = decoder.frames;
        bytes = decoder.bytes;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}

/* To change the IMU's configuration, send one command at a time and wait for
 * its "OK" line before the next, for example
 *     imu_uart_write("\r\nLOG HI91 ONTIME 0.02\r\n");    (50 Hz)
 *     imu_uart_write("\r\nSAVECONFIG\r\n");              (last) */

#elif IMU_INTERFACE == IMU_CAN

#include "imu_can.h"

#define IMU_NODE HIPNUC_CAN_NODE_DEFAULT   /* the IMU's CAN source address (default: 8) */

static hipnuc_data_t imu;   /* the latest value of each quantity: one J1939 message carries one */
static uint32_t frames;     /* CAN frames received */
static uint32_t messages;   /* HiPNUC messages from IMU_NODE */
static uint32_t others;     /* HiPNUC messages from other nodes */
static uint8_t other_node;

/* HAL calls this for every CAN controller; it hands CAN1's frames on. If your
 * project already has this callback, add the call to yours. */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    imu_can_rx_pending(hcan);
}

/* The bit rate CubeMX set: PCLK1 / prescaler / time quanta per bit. */
static uint32_t can_bitrate(void)
{
    const CAN_InitTypeDef *init = &IMU_CAN_HANDLE->Init;
    uint32_t quanta = 1U + ((init->TimeSeg1 >> CAN_BTR_TS1_Pos) + 1U) + ((init->TimeSeg2 >> CAN_BTR_TS2_Pos) + 1U);
    return HAL_RCC_GetPCLK1Freq() / (init->Prescaler * quanta);
}

static void print_summary(void)
{
    static uint32_t last_frames, last_messages, last_others;

    printf("%lu msg/s", (unsigned long)((messages - last_messages) / 2U));
    if (messages != last_messages && (imu.valid & HIPNUC_VALID_ROLL_PITCH)) {
        printf("  roll %.1f pitch %.1f", imu.roll * HIPNUC_RAD2DEG, imu.pitch * HIPNUC_RAD2DEG);
    }
    if (messages != last_messages && (imu.valid & HIPNUC_VALID_YAW)) printf(" yaw %.1f", imu.yaw * HIPNUC_RAD2DEG);
    printf(" | messages %lu frames %lu dropped %lu\n", (unsigned long)messages, (unsigned long)frames,
           (unsigned long)imu_can_dropped);
    if (frames == last_frames) {
        printf("hint: no CAN frames: check the IMU's power, CAN_H/CAN_L, 120 ohm termination, the transceiver's 3.3 V and GND, and the bit rate\n");
    } else if (messages == last_messages && others != last_others) {
        printf("hint: HiPNUC messages from node %u, not %u: set IMU_NODE\n", (unsigned)other_node, (unsigned)IMU_NODE);
    } else if (messages == last_messages) {
        printf("hint: CAN frames but no HiPNUC messages: check that the IMU outputs J1939\n");
    }
    last_frames = frames;
    last_messages = messages;
    last_others = others;
}

void app_start(void)
{
    printf("\nHiPNUC STM32F407 example: IMU on CAN1 at %lu kbit/s, node %u\n", (unsigned long)(can_bitrate() / 1000U),
           (unsigned)IMU_NODE);
    if (imu_can_start() != 0) printf("CAN did not start: PB8 (RX) must idle high; check the transceiver\n");
}

void app_poll(void)
{
    hipnuc_can_frame_t frame;
    hipnuc_data_t msg;

    while (imu_can_read(&frame)) {
        frames++;
        if (hipnuc_can_decode(&frame, &msg) != 1) continue;   /* not a HiPNUC message */
        if (msg.node_id != IMU_NODE) {
            others++;
            other_node = msg.node_id;
            continue;
        }
        messages++;
        hipnuc_data_merge(&imu, &msg);
        /* use msg here (msg.valid says which quantity it carries), or imu for the latest of all */
    }

    if (HAL_GetTick() - last >= 2000U) {
        last += 2000U;
        print_summary();
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}

#else
#error "IMU_INTERFACE: IMU_UART or IMU_CAN"
#endif

/* printf() to the console, USART1 (Keil: Use MicroLIB). */
extern UART_HandleTypeDef huart1;

static int console_putchar(int ch)
{
    uint8_t c[2] = {'\r', (uint8_t)ch};
    if (ch == '\n') HAL_UART_Transmit(&huart1, c, 2, 10);
    else HAL_UART_Transmit(&huart1, c + 1, 1, 10);
    return ch;
}

#if defined(__ARMCC_VERSION)   /* Arm Compiler 5 or 6 (version 6 defines __GNUC__ too) */
int fputc(int ch, FILE *f)
{
    (void)f;
    return console_putchar(ch);
}
#else                          /* GCC (STM32CubeIDE): the generated _write() calls this */
int __io_putchar(int ch)
{
    return console_putchar(ch);
}
#endif
