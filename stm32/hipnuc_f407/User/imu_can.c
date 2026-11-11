/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The CAN bus to the IMU, on the STM32 HAL; see imu_can.h. CubeMX sets the
 * controller up (MX_CAN1_Init()); this file adds what CubeMX does not
 * generate: the filter, the start and the receive interrupt's work.
 *
 * The receive FIFO holds only 3 frames, and the IMU sends 400 a second: the
 * interrupt moves them into a buffer, and the main loop reads them from there.
 */

#include "imu_can.h"

#define IMU IMU_CAN_HANDLE

#define BUFFER_SIZE 64U   /* frames, a power of two */

typedef struct {
    uint32_t id;
    uint8_t  extended;
    uint8_t  remote;
    uint8_t  len;
    uint8_t  data[8];
} frame_t;

static volatile frame_t buffer[BUFFER_SIZE];
static volatile uint32_t head, tail;   /* free-running counts: head is written by the interrupt, tail by the main loop */
volatile uint32_t imu_can_dropped;

int imu_can_start(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;   /* identifier and mask all zero: every frame passes */
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;             /* banks 0-13 for CAN1, 14-27 for CAN2 (FilterBank 14 there) */
    if (HAL_CAN_ConfigFilter(IMU, &filter) != HAL_OK) return -1;
    if (HAL_CAN_Start(IMU) != HAL_OK) return -1;  /* fails when RX never sees an idle bus */
    if (HAL_CAN_ActivateNotification(IMU, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) return -1;
    return 0;
}

void imu_can_rx_pending(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    uint32_t i;

    if (hcan != IMU) return;
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) return;
        if (head - tail < BUFFER_SIZE) {
            volatile frame_t *f = &buffer[head % BUFFER_SIZE];
            f->extended = header.IDE == CAN_ID_EXT;
            f->id = f->extended ? header.ExtId : header.StdId;
            f->remote = header.RTR == CAN_RTR_REMOTE;
            f->len = header.DLC > 8U ? 8U : (uint8_t)header.DLC;   /* a DLC above 8 still means 8 bytes */
            for (i = 0; i < 8U; i++) f->data[i] = data[i];
            head++;
        } else {
            imu_can_dropped++;
        }
    }
}

int imu_can_read(hipnuc_can_frame_t *frame)
{
    const volatile frame_t *f;
    uint32_t i;

    if (tail == head) return 0;
    f = &buffer[tail % BUFFER_SIZE];
    frame->id = f->id;
    frame->extended = f->extended;
    frame->remote = f->remote;
    frame->len = f->len;
    for (i = 0; i < f->len; i++) frame->data[i] = f->data[i];
    tail++;
    return 1;
}
