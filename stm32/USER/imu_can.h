/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The CAN bus to the IMU (CAN1), on the STM32 HAL. This file and imu_can.c are
 * the only CAN code that depends on the MCU: to port the example, implement
 * the two functions below with your platform's CAN driver (the counter is
 * optional) and drop the STM32 HAL part.
 */

#ifndef IMU_CAN_H
#define IMU_CAN_H

#include <stdint.h>

#include "hipnuc_can.h"

#define IMU_CAN_BITRATE 500000U   /* the IMU's factory setting; 1000000, 250000 and 125000 also work here */

/* bxCAN is classic CAN: keep the IMU's CANFD83 output off (an FD frame is an
 * error to it, for every node on the bus). */

/* ---- What the example needs from a CAN driver ---- */
int imu_can_start(void);                     /* start receiving every frame on the bus; 0 when started */
int imu_can_read(hipnuc_can_frame_t *frame); /* the next received frame: 1, or 0 when there is none */

extern volatile uint32_t imu_can_dropped;    /* frames dropped because the read buffer was full */

/* ---- STM32 HAL only ---- */
#include "stm32f4xx_hal.h"

/* Call it from your HAL_CAN_RxFifo0MsgPendingCallback() (main.c shows how);
 * it ignores other CAN controllers. */
void imu_can_rx_pending(CAN_HandleTypeDef *hcan);

#endif /* IMU_CAN_H */
