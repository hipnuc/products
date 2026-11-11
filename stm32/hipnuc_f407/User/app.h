/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The example application: reads a HiPNUC IMU and prints a summary every 2 s
 * on the console (USART1, 115200 8N1).
 */

#ifndef APP_H
#define APP_H

/* Call once, after the MX_..._Init() calls in main(). */
void app_start(void);

/* Call from the main loop, at least every 40 ms at 921600 baud. */
void app_poll(void);

#endif /* APP_H */
